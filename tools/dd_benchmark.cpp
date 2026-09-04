// Carrier-LaCAM benchmark driver.
// usage: dd_benchmark INSTANCE.yaml TIME_LIMIT_SEC PLAN_OUT [SEED] [MODE]
//   MODE: lacam (default) | b0 (rollout-only baseline) | b1 (2-stage)
//
// stdout metrics: solved, makespan, loaded_moves, free_moves, lift_drop,
//                 weighted_soc (alpha=beta=gamma=delta=1), runtime_ms,
//                 hl_nodes, hl_expanded, pibt_calls, validator_rejects,
//                 duplicate_configs, timed_out
// PLAN_OUT: one line per timestep; per-robot tokens joined by ';':
//           'w' | 'm R C' | 'l' | 'd'   (robot order = YAML robots order)
#include <dd_carrier.hpp>
#include <dd_planner.hpp>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <set>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

struct OutputPaths {
  std::string plan;
  std::string legacy_tmp;
  std::string best_effort;
  std::string best_effort_legacy_tmp;
  std::string parent;
  std::string name;

  std::vector<std::string> cleanup_slots() const
  {
    return {plan, legacy_tmp, best_effort, best_effort_legacy_tmp};
  }
};

bool split_path(const std::string& path, std::string* parent,
                std::string* name)
{
  if (path.empty() || path.back() == '/') return false;
  const auto slash = path.find_last_of('/');
  *parent = slash == std::string::npos
                ? "."
                : (slash == 0 ? "/" : path.substr(0, slash));
  *name = slash == std::string::npos ? path : path.substr(slash + 1);
  return !name->empty() && *name != "." && *name != "..";
}

std::string lexical_absolute(const std::string& path)
{
  std::string absolute = path;
  if (absolute.empty() || absolute.front() != '/') {
    std::vector<char> cwd(4096, '\0');
    while (getcwd(cwd.data(), cwd.size()) == nullptr && errno == ERANGE)
      cwd.resize(cwd.size() * 2, '\0');
    if (cwd.front() != '\0')
      absolute = std::string(cwd.data()) + "/" + absolute;
  }

  std::vector<std::string> components;
  size_t begin = 0;
  while (begin <= absolute.size()) {
    const size_t end = absolute.find('/', begin);
    const std::string part =
        absolute.substr(begin, end == std::string::npos
                                   ? std::string::npos
                                   : end - begin);
    if (!part.empty() && part != ".") {
      if (part == "..") {
        if (!components.empty()) components.pop_back();
      } else {
        components.push_back(part);
      }
    }
    if (end == std::string::npos) break;
    begin = end + 1;
  }

  std::string normalized = "/";
  for (size_t i = 0; i < components.size(); ++i) {
    if (i) normalized += "/";
    normalized += components[i];
  }
  return normalized;
}

std::string canonical_entry_path(const std::string& path)
{
  std::string parent;
  std::string name;
  if (split_path(path, &parent, &name)) {
    if (char* resolved_parent = realpath(parent.c_str(), nullptr)) {
      std::string result(resolved_parent);
      std::free(resolved_parent);
      if (result != "/") result += "/";
      result += name;
      return result;
    }
  }
  return lexical_absolute(path);
}

struct InputIdentity {
  std::set<std::string> protected_entries;
};

std::string absolute_unresolved_path(const std::string& path)
{
  if (!path.empty() && path.front() == '/') return path;
  std::vector<char> cwd(4096, '\0');
  while (getcwd(cwd.data(), cwd.size()) == nullptr && errno == ERANGE)
    cwd.resize(cwd.size() * 2, '\0');
  if (cwd.front() == '\0') return path;
  return std::string(cwd.data()) + "/" + path;
}

void prepend_components(const std::string& path,
                        std::deque<std::string>* pending)
{
  std::vector<std::string> components;
  size_t begin = 0;
  while (begin <= path.size()) {
    const size_t end = path.find('/', begin);
    const std::string part =
        path.substr(begin, end == std::string::npos
                               ? std::string::npos
                               : end - begin);
    if (!part.empty()) components.push_back(part);
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  for (auto it = components.rbegin(); it != components.rend(); ++it)
    pending->push_front(*it);
}

std::string resolved_component_path(
    const std::vector<std::string>& resolved,
    const std::string& component = "")
{
  std::string path = "/";
  for (size_t i = 0; i < resolved.size(); ++i) {
    if (i) path += "/";
    path += resolved[i];
  }
  if (!component.empty()) {
    if (path != "/") path += "/";
    path += component;
  }
  return path;
}

bool read_symlink_target(const std::string& path, std::string* target)
{
  size_t capacity = 256;
  while (capacity <= 1024 * 1024) {
    std::vector<char> buffer(capacity);
    const ssize_t length =
        readlink(path.c_str(), buffer.data(), buffer.size());
    if (length < 0) return false;
    if (static_cast<size_t>(length) < buffer.size()) {
      target->assign(buffer.data(), static_cast<size_t>(length));
      return true;
    }
    capacity *= 2;
  }
  return false;
}

InputIdentity identify_input(const std::string& path)
{
  InputIdentity identity;
  identity.protected_entries.insert(canonical_entry_path(path));

  const std::string absolute = absolute_unresolved_path(path);
  std::deque<std::string> pending;
  prepend_components(absolute, &pending);
  std::vector<std::string> resolved;
  size_t symlink_count = 0;
  bool complete = true;

  while (!pending.empty()) {
    const std::string component = pending.front();
    pending.pop_front();
    if (component == ".") continue;
    if (component == "..") {
      if (!resolved.empty()) resolved.pop_back();
      continue;
    }

    const std::string entry =
        resolved_component_path(resolved, component);
    identity.protected_entries.insert(entry);
    struct stat info;
    if (lstat(entry.c_str(), &info) != 0) {
      complete = false;
      break;
    }
    if (!S_ISLNK(info.st_mode)) {
      resolved.push_back(component);
      continue;
    }

    std::string target;
    if (++symlink_count > 64 ||
        !read_symlink_target(entry, &target)) {
      complete = false;
      break;
    }
    if (!target.empty() && target.front() == '/') resolved.clear();
    prepend_components(target, &pending);
  }

  if (complete)
    identity.protected_entries.insert(resolved_component_path(resolved));
  if (char* final_target = realpath(path.c_str(), nullptr)) {
    identity.protected_entries.insert(final_target);
    std::free(final_target);
  }
  return identity;
}

bool is_input_entry(const InputIdentity& input, const std::string& path)
{
  return input.protected_entries.count(canonical_entry_path(path)) != 0;
}

void append_error(std::string* error, const std::string& message)
{
  if (!error->empty()) *error += "; ";
  *error += message;
}

bool unlink_with_retry(const std::string& path)
{
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (unlink(path.c_str()) == 0 || errno == ENOENT) return true;
  }
  return false;
}

bool clear_output_slot(const InputIdentity& input,
                       const std::string& slot, std::string* error)
{
  struct stat info;
  if (lstat(slot.c_str(), &info) == 0) {
    if (is_input_entry(input, slot)) {
      append_error(error, "output path is INSTANCE.yaml: " + slot);
      return false;
    }
    if (!S_ISREG(info.st_mode) && !S_ISLNK(info.st_mode)) {
      append_error(error, "output slot is not removable file: " + slot);
      return false;
    }
    if (!unlink_with_retry(slot)) {
      append_error(error, "cannot remove stale output: " + slot);
      return false;
    }
  } else if (errno != ENOENT) {
    append_error(error, "cannot inspect output slot: " + slot);
    return false;
  }
  return true;
}

bool prepare_output_paths(const std::string& plan_out, OutputPaths* paths,
                          std::string* error)
{
  if (!split_path(plan_out, &paths->parent, &paths->name)) {
    *error = "PLAN_OUT must name a file";
    return false;
  }
  struct stat parent_info;
  if (stat(paths->parent.c_str(), &parent_info) != 0 ||
      !S_ISDIR(parent_info.st_mode)) {
    *error =
        "PLAN_OUT parent is not an existing directory: " + paths->parent;
    return false;
  }

  paths->plan = plan_out;
  paths->legacy_tmp = plan_out + ".tmp";
  paths->best_effort = plan_out + ".best_effort";
  paths->best_effort_legacy_tmp = paths->best_effort + ".tmp";
  return true;
}

bool starts_with(const std::string& value, const std::string& prefix)
{
  return value.size() >= prefix.size() &&
         value.compare(0, prefix.size(), prefix) == 0;
}

std::string join_path(const std::string& parent, const std::string& name)
{
  return parent == "/" ? "/" + name : parent + "/" + name;
}

bool clear_orphan_temps(const InputIdentity& input,
                        const std::string& final_path, std::string* error)
{
  std::string parent;
  std::string name;
  if (!split_path(final_path, &parent, &name)) return false;
  DIR* dir = opendir(parent.c_str());
  if (dir == nullptr) {
    append_error(error, "cannot scan output directory: " + parent);
    return false;
  }

  const std::string legacy_prefix = name + ".tmp.";
  const std::string owned_prefix =
      name + ".ddbench-tmp.u" + std::to_string(geteuid()) + ".";
  bool ok = true;
  while (const dirent* entry = readdir(dir)) {
    const std::string entry_name = entry->d_name;
    if (!starts_with(entry_name, legacy_prefix) &&
        !starts_with(entry_name, owned_prefix))
      continue;
    const std::string path = join_path(parent, entry_name);
    struct stat info;
    if (lstat(path.c_str(), &info) != 0) {
      if (errno != ENOENT) {
        append_error(error, "cannot inspect orphan temp: " + path);
        ok = false;
      }
      continue;
    }
    if (is_input_entry(input, path)) {
      append_error(error, "orphan temp path is INSTANCE.yaml: " + path);
      ok = false;
      continue;
    }
    const bool owned_regular =
        S_ISREG(info.st_mode) && info.st_uid == geteuid() &&
        (info.st_mode & 0777) == 0600;
    if (!owned_regular) {
      append_error(error, "orphan temp is not owned regular file: " + path);
      ok = false;
      continue;
    }
    if (!unlink_with_retry(path)) {
      append_error(error, "cannot remove orphan temp: " + path);
      ok = false;
    }
  }
  if (closedir(dir) != 0) {
    append_error(error, "cannot close output directory: " + parent);
    ok = false;
  }
  return ok;
}

bool clear_plan_outputs(const std::string& yaml_path,
                        const OutputPaths& paths, std::string* error)
{
  const InputIdentity input = identify_input(yaml_path);
  bool ok = true;
  for (const auto& slot : paths.cleanup_slots()) {
    if (!clear_output_slot(input, slot, error)) ok = false;
  }
  if (!clear_orphan_temps(input, paths.plan, error)) ok = false;
  if (!clear_orphan_temps(input, paths.best_effort, error)) ok = false;
  return ok;
}

bool parse_time_limit(const std::string& text, double* value)
{
  try {
    size_t consumed = 0;
    *value = std::stod(text, &consumed);
    return consumed == text.size() && std::isfinite(*value) && *value >= 0;
  } catch (const std::exception&) {
    return false;
  }
}

bool parse_seed(const std::string& text, int* value)
{
  try {
    size_t consumed = 0;
    const long long parsed = std::stoll(text, &consumed);
    if (consumed != text.size() ||
        parsed < std::numeric_limits<int>::min() ||
        parsed > std::numeric_limits<int>::max())
      return false;
    *value = static_cast<int>(parsed);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

std::string serialize_plan(const DDPlan& plan, const DDInstance& ins)
{
  std::ostringstream out;
  for (const auto& ops : plan) {
    for (size_t i = 0; i < ops.size(); ++i) {
      if (i) out << ";";
      switch (ops[i].kind) {
        case Op::WAIT: out << "w"; break;
        case Op::MOVE:
          out << "m " << ins.grid.row(ops[i].to) << " "
              << ins.grid.col(ops[i].to);
          break;
        case Op::LIFT: out << "l"; break;
        case Op::DROP: out << "d"; break;
      }
    }
    out << "\n";
  }
  return out.str();
}

bool write_all_and_sync(int fd, const std::string& contents)
{
  size_t written = 0;
  while (written < contents.size()) {
    const ssize_t count =
        write(fd, contents.data() + written, contents.size() - written);
    if (count > 0) {
      written += static_cast<size_t>(count);
    } else if (count < 0 && errno == EINTR) {
      continue;
    } else {
      return false;
    }
  }
  return fsync(fd) == 0;
}

enum class PublishResult { SUCCESS, UNSUPPORTED, FAILURE };

PublishResult publish_from_unnamed_temp(
    const std::string& contents, const std::string& parent,
    const std::string& plan_out)
{
#ifdef O_TMPFILE
  const int fd =
      open(parent.c_str(), O_TMPFILE | O_WRONLY | O_CLOEXEC, 0600);
  if (fd < 0) {
    if (errno == EOPNOTSUPP || errno == EINVAL || errno == ENOSYS)
      return PublishResult::UNSUPPORTED;
    return PublishResult::FAILURE;
  }
  if (!write_all_and_sync(fd, contents)) {
    close(fd);
    return PublishResult::FAILURE;
  }

  bool published =
      linkat(fd, "", AT_FDCWD, plan_out.c_str(), AT_EMPTY_PATH) == 0;
  if (!published) {
    const std::string proc_path = "/proc/self/fd/" + std::to_string(fd);
    published =
        linkat(AT_FDCWD, proc_path.c_str(), AT_FDCWD, plan_out.c_str(),
               AT_SYMLINK_FOLLOW) == 0;
  }
  const int link_error = errno;
  const bool closed = close(fd) == 0;
  if (published && closed) return PublishResult::SUCCESS;
  if (published) unlink_with_retry(plan_out);
  if (!published &&
      (link_error == EOPNOTSUPP || link_error == EINVAL ||
       link_error == ENOSYS || link_error == EPERM ||
       link_error == ENOENT))
    return PublishResult::UNSUPPORTED;
  return PublishResult::FAILURE;
#else
  (void)contents;
  (void)parent;
  (void)plan_out;
  return PublishResult::UNSUPPORTED;
#endif
}

bool publish_from_named_temp(const std::string& contents,
                             const std::string& plan_out)
{
  std::string pattern = plan_out + ".ddbench-tmp.u" +
                        std::to_string(geteuid()) + ".XXXXXX";
  std::vector<char> writable_pattern(pattern.begin(), pattern.end());
  writable_pattern.push_back('\0');
  const int fd = mkstemp(writable_pattern.data());
  if (fd < 0) return false;
  const std::string tmp(writable_pattern.data());
  bool ok = write_all_and_sync(fd, contents);
  if (close(fd) != 0) ok = false;
  if (!ok || rename(tmp.c_str(), plan_out.c_str()) != 0) {
    unlink_with_retry(tmp);
    return false;
  }
  return true;
}

bool write_plan_atomically(const DDPlan& plan, const DDInstance& ins,
                           const std::string& plan_out)
{
  const std::string contents = serialize_plan(plan, ins);
  std::string parent;
  std::string name;
  if (!split_path(plan_out, &parent, &name)) return false;
  const PublishResult unnamed =
      publish_from_unnamed_temp(contents, parent, plan_out);
  if (unnamed == PublishResult::SUCCESS) return true;
  if (unnamed == PublishResult::FAILURE) return false;
  return publish_from_named_temp(contents, plan_out);
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 4) {
    std::cerr << "usage: dd_benchmark INSTANCE.yaml TIME_LIMIT_SEC PLAN_OUT "
                 "[SEED]\n";
    return 2;
  }
  const std::string yaml_path = argv[1];
  const std::string time_limit_text = argv[2];
  const std::string plan_out = argv[3];
  const std::string seed_text = argc >= 5 ? argv[4] : "0";
  const std::string mode = argc >= 6 ? argv[5] : "lacam";

  OutputPaths output_paths;
  std::string output_error;
  if (!prepare_output_paths(plan_out, &output_paths, &output_error)) {
    std::cerr << output_error << "\n";
    return 2;
  }
  if (!clear_plan_outputs(yaml_path, output_paths, &output_error)) {
    std::cerr << output_error << "\n";
    return 2;
  }

  double time_limit_sec = 0;
  int seed = 0;
  if (!parse_time_limit(time_limit_text, &time_limit_sec)) {
    std::cerr << "invalid TIME_LIMIT_SEC: " << time_limit_text << "\n";
    return 2;
  }
  if (!parse_seed(seed_text, &seed)) {
    std::cerr << "invalid SEED: " << seed_text << "\n";
    return 2;
  }
  if (mode != "lacam" && mode != "b0" && mode != "b1") {
    std::cerr << "unknown MODE '" << mode
              << "' (expected lacam | b0 | b1)" << std::endl;
    return 2;  // fail loudly: silent fallback masks typos (debug.md P0-4)
  }
  if (std::signal(SIGXFSZ, SIG_IGN) == SIG_ERR) {
    std::cerr << "failed to install SIGXFSZ handler\n";
    return 2;
  }

  DDInstance ins;
  try {
    ins = load_dd_instance(yaml_path);
  } catch (const std::exception& e) {
    std::cerr << "failed to load instance: " << e.what() << "\n";
    std::cout << "valid_instance=0\n";
    return 1;
  }
  std::cout << "valid_instance=1\n";

  DDSocWeights w;
  try {
    w = dd_load_soc_weights();
  } catch (const std::exception& e) {
    std::cerr << "invalid objective weights: " << e.what() << "\n";
    return 2;
  }

  const auto t0 = std::chrono::steady_clock::now();
  DDStats stats;
  DDPlan best_effort;
  DDPlan plan;
  if (mode == "b0")
    plan = solve_carrier_rollout(ins, time_limit_sec, seed, &stats);
  else if (mode == "b1")
    plan = solve_carrier_2stage(ins, time_limit_sec, seed, &stats);
  else
    plan = solve_carrier_lacam(
        ins, time_limit_sec, seed, &stats,
        std::getenv("DD_DEBUG_DUMP") ? &best_effort : nullptr);
  if (plan.empty() && !best_effort.empty()) {
    const std::string& best_effort_out = output_paths.best_effort;
    if (!write_plan_atomically(best_effort, ins, best_effort_out))
      std::cerr << "failed to write best-effort plan to " << best_effort_out
                << "\n";
  }
  const double runtime_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t0)
          .count();

  long loaded_moves = 0, free_moves = 0, lift_drop = 0, anon_moves = 0;
  bool valid = !plan.empty();
  if (valid) {
    // replay through the validator: belt-and-braces before reporting
    auto s = initial_phys_config(ins);
    for (const auto& ops : plan) {
      for (size_t i = 0; i < ops.size(); ++i) {
        switch (ops[i].kind) {
          case Op::WAIT:
            break;
          case Op::MOVE:
            if (s.kappa[i] == KAPPA_FREE) {
              ++free_moves;
            } else {
              ++loaded_moves;
              if (s.kappa[i] == KAPPA_ANON) ++anon_moves;
            }
            break;
          case Op::LIFT:
            ++lift_drop;
            break;
          case Op::DROP:
            ++lift_drop;
            break;
        }
      }
      auto nxt = apply_ops(ins, s, ops);
      if (!nxt.has_value()) {
        valid = false;
        break;
      }
      s = *nxt;
    }
    if (valid && !is_dd_goal(ins, s)) valid = false;
  }
  if (valid && !write_plan_atomically(plan, ins, plan_out)) {
    std::cerr << "failed to write validated plan to " << plan_out << "\n";
    valid = false;
  }
  if (!valid) {
    unlink(plan_out.c_str());
  }

  // cost weights (design 2.3): alpha*loaded + beta*free + gamma*liftdrop
  // + delta*anon; defaults all 1, overridable via env (DD_ALPHA etc.).
  // R4 (debug.md §10): the ONE validated parser — a private atof here
  // silently accepted garbage and bypassed the weight validation.
  const double alpha = w.alpha, beta = w.beta, gamma = w.gamma,
               delta = w.delta;
  const double weighted_soc = alpha * loaded_moves + beta * free_moves +
                              gamma * lift_drop + delta * anon_moves;

  std::cout << "mode=" << mode << "\n";
  std::cout << "solved=" << (valid ? 1 : 0) << "\n";
  std::cout << "makespan=" << (valid ? plan.size() : 0) << "\n";
  std::cout << "loaded_moves=" << loaded_moves << "\n";
  std::cout << "free_moves=" << free_moves << "\n";
  std::cout << "lift_drop=" << lift_drop << "\n";
  std::cout << "anon_moves=" << anon_moves << "\n";
  std::cout << "weighted_soc=" << weighted_soc << "\n";
  std::cout << "runtime_ms=" << runtime_ms << "\n";
  std::cout << "hl_nodes=" << stats.hl_nodes << "\n";
  std::cout << "hl_expanded=" << stats.hl_expanded << "\n";
  std::cout << "pibt_calls=" << stats.pibt_calls << "\n";
  std::cout << "validator_rejects=" << stats.validator_rejects << "\n";
  std::cout << "g1_rejects=" << stats.g1_rejects << "\n";
  std::cout << "generator_failures=" << stats.generator_failures << "\n";
  std::cout << "max_depth=" << stats.max_depth << "\n";
  std::cout << "guidance_builds=" << stats.guidance_builds << "\n";
  std::cout << "tau_time_ms=" << stats.tau_time_ms << "\n";
  std::cout << "guidance_time_ms=" << stats.guidance_time_ms << "\n";
  std::cout << "upper_epoch_builds=" << stats.upper_epoch_builds << "\n";
  std::cout << "pair_cache_hits=" << stats.pair_cache_hits << "\n";
  std::cout << "pair_cache_misses=" << stats.pair_cache_misses << "\n";
  std::cout << "pair_rollout_steps=" << stats.pair_rollout_steps << "\n";
  std::cout << "pair_rollout_truncations="
            << stats.pair_rollout_truncations << "\n";
  std::cout << "pair_rollout_stalls=" << stats.pair_rollout_stalls << "\n";
  std::cout << "tau_guide_changes_on_upper_move="
            << stats.tau_guide_changes_on_upper_move << "\n";
  std::cout << "joint_task_nodes=" << stats.joint_task_nodes << "\n";
  std::cout << "joint_task_edges=" << stats.joint_task_edges << "\n";
  std::cout << "joint_shared_effects=" << stats.joint_shared_effects
            << "\n";
  std::cout << "joint_effect_conflicts=" << stats.joint_effect_conflicts
            << "\n";
  std::cout << "joint_candidate_backtracks="
            << stats.joint_candidate_backtracks << "\n";
  std::cout << "joint_paused_roots=" << stats.joint_paused_roots << "\n";
  std::cout << "ready_task_count=" << stats.ready_task_count << "\n";
  std::cout << "rho_repairs=" << stats.rho_repairs << "\n";
  std::cout << "custody_continuations=" << stats.custody_continuations
            << "\n";
  std::cout << "zero_empty_no_ready=" << stats.zero_empty_no_ready << "\n";
  std::cout << "rewire_guidance_rebuilds="
            << stats.rewire_guidance_rebuilds << "\n";
  std::cout << "deliverable_ms=" << stats.deliverable_ms << "\n";
  std::cout << "robot_only_successors=" << stats.robot_only_successors
            << "\n";
  std::cout << "manipulation_successors=" << stats.manipulation_successors
            << "\n";
  std::cout << "shelf_motion_successors=" << stats.shelf_motion_successors
            << "\n";
  std::cout << "macro_successors=" << stats.macro_successors << "\n";
  std::cout << "macro_steps=" << stats.macro_steps << "\n";
  std::cout << "macro_shelf_motion_successors="
            << stats.macro_shelf_motion_successors << "\n";
  std::cout << "macro_robot_only_successors="
            << stats.macro_robot_only_successors << "\n";
  std::cout << "rollout_calls=" << stats.rollout_calls << "\n";
  std::cout << "rollout_cycles=" << stats.rollout_cycles << "\n";
  std::cout << "rollout_shelf_motion_steps="
            << stats.rollout_shelf_motion_steps << "\n";
  std::cout << "best_targets_done=" << stats.best_targets_done << "\n";
  std::cout << "duplicate_configs=" << stats.duplicate_configs << "\n";
  std::cout << "timed_out=" << (stats.timed_out ? 1 : 0) << "\n";
  std::cout << "seed=" << seed << "\n";
  std::cout << "mode=" << mode << "\n";
  std::cout << "first_solution_ms=" << stats.first_solution_ms << "\n";
  std::cout << "first_solution_soc=" << stats.first_solution_soc << "\n";
  std::cout << "best_soc=" << stats.best_soc << "\n";
  std::cout << "incumbent_updates=" << stats.incumbent_updates << "\n";
  std::cout << "exact_loops=" << stats.exact_loops << "\n";
  std::cout << "projected_loops=" << stats.projected_loops << "\n";
  std::cout << "bridge_steps=" << stats.bridge_steps << "\n";
  std::cout << "plan_steps_removed=" << stats.plan_steps_removed << "\n";
  std::cout << "assignment_restarts=" << stats.assignment_restarts << "\n";
  std::cout << "assignment_second_solved="
            << stats.assignment_second_solved << "\n";
  std::cout << "assignment_improvements=" << stats.assignment_improvements
            << "\n";
  std::cout << "assignment_second_solution_ms="
            << stats.assignment_second_solution_ms << "\n";
  std::cout << "assignment_first_soc=" << stats.assignment_first_soc << "\n";
  std::cout << "assignment_second_soc=" << stats.assignment_second_soc
            << "\n";
  std::cout << "assignment_first_makespan="
            << stats.assignment_first_makespan << "\n";
  std::cout << "assignment_second_makespan="
            << stats.assignment_second_makespan << "\n";
  if (!valid && std::getenv("DD_DEBUG_DUMP") &&
      !stats.deepest_config.robots.empty()) {
    const auto& X = stats.deepest_config;
    std::cerr << "--- deepest config (depth=" << stats.max_depth << ") ---\n";
    // upper occupancy set
    std::set<int> upper(X.anon_occ.begin(), X.anon_occ.end());
    for (size_t b = 0; b < ins.n_targets(); ++b) upper.insert(X.target_pos[b]);
    std::map<int, int> robot_at;
    for (size_t i = 0; i < ins.n_robots(); ++i)
      robot_at[X.robots[i]] = (int)i;
    for (size_t b = 0; b < ins.n_targets(); ++b) {
      bool carried = false;
      for (int k : X.kappa) carried |= (k == (int)b);
      const int p = X.target_pos[b];
      const int gcell =
          b < stats.deepest_tau.size() ? stats.deepest_tau[b]
                                      : ins.target_goals[b];
      if (!carried && p == gcell) continue;
      std::cerr << "target b" << b << " at (" << ins.grid.row(p) << ","
                << ins.grid.col(p) << ") goal (" << ins.grid.row(gcell) << ","
                << ins.grid.col(gcell) << ") carried=" << carried << "\n";
      const int r0 = std::max(0, ins.grid.row(gcell) - 3);
      const int r1 = std::min(ins.grid.height - 1, ins.grid.row(gcell) + 3);
      const int c0 = std::max(0, ins.grid.col(gcell) - 3);
      const int c1 = std::min(ins.grid.width - 1, ins.grid.col(gcell) + 3);
      for (int r = r0; r <= r1; ++r) {
        std::string line;
        for (int c = c0; c <= c1; ++c) {
          const int v = ins.grid.idx(r, c);
          char ch = '.';
          if (ins.grid.is_wall(v)) ch = '@';
          else if (v == gcell) ch = upper.count(v) ? 'X' : 'G';
          else if (v == p) ch = 'B';
          else if (upper.count(v)) ch = '#';
          if (robot_at.count(v) && ch == '.') ch = 'r';
          line += ch;
        }
        std::cerr << "  " << line << "\n";
      }
    }
    for (size_t i = 0; i < ins.n_robots(); ++i)
      std::cerr << "robot " << i << " at (" << ins.grid.row(X.robots[i])
                << "," << ins.grid.col(X.robots[i]) << ") kappa="
                << X.kappa[i] << "\n";
  }
  return 0;
}
