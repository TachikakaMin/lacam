import unittest

from generate_web_viz import TEMPLATE


class WebVizResponsiveTest(unittest.TestCase):
    def test_animation_template_is_mobile_responsive(self):
        self.assertIn(
            '<meta name="viewport" '
            'content="width=device-width,initial-scale=1">',
            TEMPLATE,
        )
        self.assertIn("@media(max-width:600px)", TEMPLATE)
        self.assertIn("max-width: 100%", TEMPLATE)
        self.assertIn("height: auto", TEMPLATE)
        self.assertNotIn("width: 320px", TEMPLATE)
        self.assertNotIn('style="width:360px"', TEMPLATE)


if __name__ == "__main__":
    unittest.main()
