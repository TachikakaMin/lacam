(() => {
  'use strict';
  // Translate content in place so the player, controls, and their listeners survive a switch.
  const translations = [
    ['nav .links a', ['摘要', '交互演示', '建造成品', '方法', '实验结果', '引用']],
    ['.kicker', ['研究论文 · arXiv:2609.14208 · cs.RO']],
    ['h1', ['ITA-LaCAM：通过融合目标分配的配置空间搜索，实现完备、可扩展的 TAPF 求解']],
    ['.affil', ['<sup>1</sup>南加州大学 &nbsp;·&nbsp; <sup>2</sup>加州大学欧文分校 &nbsp;·&nbsp; <sup>3</sup>Symbotic']],
    ['.btnrow a', ['论文', 'HTML 全文', '交互演示 ↓']],
    ['#abstract h2', ['摘要']],
    ['#abstract .narrow > p', [
      '目标分配与路径规划（Combined Target Assignment and Path Finding，TAPF）需要同时决定每台机器人去哪个目标，以及如何无碰撞地到达。',
      '我们提出 <strong>ITA-LaCAM</strong>，一个受 LaCAM 和 ITA-CBS 启发、具有完备性和可扩展性的 TAPF 求解器。每个联合配置节点都带有一份机器人与目标的匹配关系。生成后继配置时，算法只针对发生移动的机器人增量修复匹配，并用更新后的目标引导 PIBT 生成下一步动作。这样既能在搜索过程中灵活调整目标，也无需显式枚举所有分配组合，同时保留 LaCAM 的完备性和可扩展性。',
      '在八张地图、5–200 台机器人的 9,760 个测试实例上，ITA-LaCAM 的成功率为 <strong>100%</strong>，IR-TAPF（DBS-Hungarian）为 95.6%。ITA-LaCAM 在 <strong>84.0%</strong> 的实例中更早找到首个可行解；在双方都解出的实例中，有 <strong>65.0%</strong> 的结果具有更低的总代价（SOC）。'
    ]],
    ['.stat .lbl', [
      '八张地图上的 9,760 个实例，全部在 10 秒时限内求解成功',
      '在这些实例中，比 IR-TAPF 更早找到首个可行解',
      '在双方都解出的实例中，返回的路径总代价更低',
      '与 ITA-CBS 共同解出的实例中，90% 的结果与最优解之差不超过这一比例'
    ]],
    ['#demos h2', ['交互演示：多机器人协同建造']],
    ['#demos .wrap > p', [
      '下面的建造仿真将 TAPF 求解器用于逐轮规划：根据当前建造状态，整理搬运、可放置砖块和脚手架拆除等任务，求解后执行各机器人的路径。机器人每步的高度变化满足 |Δh| ≤ 1，只能从高度合适的相邻位置放砖；悬空砖块需要侧面连接或悬挂支撑，脚手架最终会被拆除并送回料仓。拖动可旋转视角，滚轮可缩放。'
    ]],
    ['#demos .hint', ['演示回放规划器生成的轨迹。拖动旋转 · 滚轮或双指缩放 · 按住 ⌘ / Ctrl 拖动（或按住鼠标中键拖动）平移 · 双击回到中心。可用上方时间轴和速度控件调整播放。']],
    ['#gallery h2', ['建造成品']],
    ['#gallery .wrap > p', [
      '机器人按照 BrickGPT 风格的砖块清单逐块搭建，包括来自 <a class="inline" href="https://avalovelace1.github.io/BrickGPT/">StableText2Brick</a> 的体素设计和 MOC 搭建说明，并完成脚手架的搭设与拆除。总完成时间（makespan，T）从首次取砖开始，计至最后一块脚手架归还料仓，以仿真时间步计。',
      '所有演示轨迹都通过独立校验：每步移动满足 |Δh| ≤ 1，无同格碰撞或对向换位碰撞；放砖位置与站位合法；取砖、放砖、拆除和回库数量守恒；最终结构与目标完全一致。'
    ]],
    ['#gallery th', ['结构', '放置砖块', '脚手架砖块', '机器人', '完成时间 T']],
    ['#gallery tbody td:first-child', ['积木汽车', '积木钢琴', '维多利亚式马车（含积木马）', '积木吉他', '积木椅子']],
    ['#method h2', ['方法']],
    ['#method .narrow > p', [
      'LaCAM* 在<em>联合配置空间</em>中进行类似 A* 的搜索。一个配置记录所有机器人的当前位置；算法用优先级继承与回溯（Priority Inheritance with Backtracking，PIBT）按需生成后继配置，从而探索合法配置图，而不必一次枚举所有联合动作。关键在于：下一步是否合法，只取决于当前配置，而不取决于到达这里的过程。因此，从固定目标的多机器人路径规划（MAPF）扩展到 TAPF，可以沿用原有搜索框架，加入<strong>随配置变化的目标分配引导</strong>。',
      'CBS-TA 和 IR-TAPF 先固定目标分配，完成一次 MAPF 求解后才能获得整条路径的反馈。ITA-LaCAM 则在<em>每个搜索节点</em>更新匹配，让目标分配与路径搜索及时交换信息。'
    ]],
    ['.mcard b', ['每个节点都有目标匹配', '增量修复匹配', '带 Swap 机制的 PIBT', '完备性与 anytime 搜索']],
    ['.mcard p', [
      '每个高层节点都用 Hungarian（匈牙利）算法计算可行的最小距离匹配，距离暂不考虑机器人之间的碰撞。匹配结果提供 PIBT 的临时目标、启发式估计，以及依赖当前分配的搜索代价。',
      '生成新配置后，只有发生移动的机器人对应的代价矩阵行会变化。ITA-LaCAM 复用父节点的匹配和 Hungarian 对偶变量，只修复这些行，无需在每个节点从头求解。',
      'PIBT 通过优先级继承协调动作，并用 Swap 机制处理狭窄通道中的反复阻塞：需要错身时，一台机器人先让开，另一台再进入腾出的格子，通过多步合法移动完成避让。',
      '目标分配负责引导搜索，不会永久排除合法后继；按需扩展仍能遍历可达配置，保留 LaCAM* 的完备性。找到首个可行解后，anytime 搜索继续利用剩余时间，按内部搜索代价保留当前最佳方案。'
    ]],
    ['#results h2', ['实验结果']],
    ['.results-intro p', ['9,760 个测试实例，八张地图，每个实例限时 10 秒。<br>ITA-LaCAM 全部求解成功。']],
    ['.figure-label', ['图 4 · 首解时间', '图 5 · 解的质量', '图 6 · 与最优解的差距']],
    ['.evidence-card h3', ['84% 的实例更快出解。', '65% 的实例总代价更低。', '中位代价仅比最优高 1.6%。']],
    ['.evidence-card figcaption > p:not(.reading)', [
      '在 <strong>9,760 个测试实例</strong>中，ITA-LaCAM 有 <strong>84.0%</strong> 比 IR-TAPF 更早找到首个可行解。IR-TAPF 有 425 个实例超时，ITA-LaCAM 则全部解出。',
      '在双方都解出的 <strong>9,335 个实例</strong>中，ITA-LaCAM 有 <strong>65.0%</strong> 的结果优于 IR-TAPF。指标 SOC 是所有机器人的到达时间之和，包括移动和等待。',
      '在与最优求解器 ITA-CBS 共同解出的 <strong>4,955 个实例</strong>中，ITA-LaCAM 相对最优解的代价差距<strong>中位数为 1.6%</strong>，90% 的结果差距不超过 10.2%。'
    ]],
    ['.evidence-card .reading', [
      '每个点代表一个实例；在对角线下方，表示 ITA-LaCAM 更快。',
      '纵轴为 IR-TAPF SOC / ITA-LaCAM SOC；高于 1，表示 ITA-LaCAM 代价更低。',
      '纵轴为 ITA-LaCAM SOC / ITA-CBS SOC；越接近 1，越接近最优。仅比较双方都解出的实例。'
    ]],
    ['.results-source', ['图 4–6 来自<a class="inline" href="https://arxiv.org/html/2609.14208v1">论文原文</a>，点击可放大。原图坐标与算法名称保留英文，授权协议为 CC BY-SA 4.0。']],
    ['#citation h2', ['引用']]
  ];
  const entries = translations.flatMap(([selector, zh]) => {
    const nodes = [...document.querySelectorAll(selector)];
    if (nodes.length !== zh.length) throw new Error(`Translation count mismatch: ${selector}`);
    return nodes.map((node, i) => ({ node, en: node.innerHTML, zh: zh[i] }));
  });
  const sceneNames = {
    'Arch Bridge': '拱桥', 'Column': '立柱', 'USC Gate': 'USC 校门', 'Pyramid': '金字塔',
    'Giza Site': '吉萨建筑群', 'Brick Car': '积木汽车', 'SymBot': 'SymBot 搬运机器人',
    'Temple': '神庙', 'Colonnade': '柱廊神庙', 'Victorian Brougham': '维多利亚式马车',
    'Trojan Horse': '木马', 'Tommy Trojan': 'Tommy Trojan 雕像', 'Brick Piano': '积木钢琴',
    'Golden Gate Bridge': '金门大桥', 'Brick Guitar': '积木吉他', 'Brick Chair': '积木椅子'
  };
  const galleryNotes = {
    'T = 7,526 · validator PASS': 'T = 7,526 · 校验通过',
    'T = 5,382 · validator PASS': 'T = 5,382 · 校验通过',
    'T = 3,624 · validator PASS': 'T = 3,624 · 校验通过',
    'T = 792 · validator PASS': 'T = 792 · 校验通过',
    'T = 1,394 · brick-built horse': 'T = 1,394 · 含积木马',
    'suspended cable erection': '悬空缆索搭建', 'ascending float-chain sword': '逐段向上搭建悬空剑身',
    'case-carrying drive unit': '货箱搬运机器人'
  };
  document.querySelectorAll('#demoChips .chip, #galleryGrid .cap b, #galleryGrid .cap span').forEach(node => {
    const en = node.textContent;
    const zh = sceneNames[en] || galleryNotes[en];
    if (!zh) throw new Error(`Missing translation: ${en}`);
    entries.push({ node, en: node.innerHTML, zh });
  });
  const attributeEntries = [];
  function attributes(selector, attribute, zh) {
    document.querySelectorAll(selector).forEach((node, i) => {
      attributeEntries.push({ node, attribute, en: node.getAttribute(attribute), zh: Array.isArray(zh) ? zh[i] : zh });
    });
  }
  attributes('.evidence', 'aria-label', '论文图 4、5、6');
  attributes('.figure-link', 'aria-label', ['放大查看图 4', '放大查看图 5', '放大查看图 6']);
  attributes('.evidence-card img', 'alt', [
    '首解时间对比：横轴为 IR-TAPF，纵轴为 ITA-LaCAM；对角线下方表示 ITA-LaCAM 更快。',
    'IR-TAPF SOC 与 ITA-LaCAM SOC 的比值；高于 1 表示 ITA-LaCAM 代价更低。',
    'ITA-LaCAM SOC 与最优 ITA-CBS SOC 的比值；1 表示最优代价。'
  ]);
  document.querySelectorAll('#galleryGrid img').forEach(node => {
    attributeEntries.push({ node, attribute: 'alt', en: node.alt, zh: sceneNames[node.alt] });
  });
  attributes('#demoFrame', 'title', 'LaCAM-TAPF 多机器人建造演示');
  const frame = document.getElementById('demoFrame');
  const player = document.getElementById('player');
  let language = 'en';
  try { language = localStorage.getItem('itaLacamLanguage') || (navigator.language.startsWith('zh') ? 'zh' : 'en'); } catch {}
  if (language !== 'zh') language = 'en';
  const englishTitle = document.title;
  function label(id, en, zh) {
    const node = document.getElementById(id);
    node.title = language === 'zh' ? zh : en;
    node.setAttribute('aria-label', node.title);
  }
  window.syncLanguageControls = () => {
    const zh = language === 'zh';
    const tall = player.classList.contains('tall');
    document.getElementById('growBtn').textContent = tall ? (zh ? '⤡ 还原' : '⤡ Restore') : (zh ? '⤢ 放大' : '⤢ Expand');
    document.getElementById('fsBtn').textContent = zh ? '⛶ 全屏' : '⛶ Fullscreen';
    document.getElementById('newTabBtn').textContent = zh ? '↗ 新标签页' : '↗ New tab';
    document.getElementById('speedLabel').textContent = zh ? '速度' : 'Speed';
    label('growBtn', tall ? 'Restore player height' : 'Expand player', tall ? '还原播放器高度' : '放大播放器');
    label('fsBtn', 'Fullscreen', '全屏');
    label('newTabBtn', 'Open scene in a new tab', '在新标签页打开场景');
    label('ctlPlay', 'Play / pause', '播放 / 暂停');
    label('ctlReset', 'Restart', '重新播放');
    label('ctlTimeline', 'Playback position', '播放进度');
    label('ctlSpeed', 'Playback speed', '播放速度');
  };
  function syncFrame() {
    frame.contentWindow?.postMessage({ lacamCmd: { cmd: 'language', value: language } }, location.origin);
  }
  function applyLanguage(next) {
    language = next;
    document.documentElement.lang = next === 'zh' ? 'zh-CN' : 'en';
    document.title = next === 'zh' ? 'ITA-LaCAM：目标分配与路径规划 · 多机器人建造演示' : englishTitle;
    entries.forEach(entry => { entry.node.innerHTML = entry[next]; });
    attributeEntries.forEach(entry => { entry.node.setAttribute(entry.attribute, entry[next]); });
    document.querySelectorAll('[data-language]').forEach(button => {
      button.setAttribute('aria-pressed', String(button.dataset.language === next));
    });
    try {
      localStorage.setItem('itaLacamLanguage', next);
      localStorage.setItem('demoLang', next);
    } catch {}
    window.syncLanguageControls();
    syncFrame();
  }
  document.querySelectorAll('[data-language]').forEach(button => {
    button.addEventListener('click', () => applyLanguage(button.dataset.language));
  });
  frame.addEventListener('load', syncFrame);
  applyLanguage(language);
})();
