import copy
import unittest

from check_pr import validate


class PRPolicyTests(unittest.TestCase):
    def setUp(self):
        self.pr = {
            "title": "feat(stm32): 增加编码器采样",
            "base": {"ref": "main"},
            "head": {"ref": "feat/12-encoder"},
            "user": {"login": "developer"},
        }

    def test_normal_and_codex_branches(self):
        self.assertEqual(validate(self.pr), [])
        self.pr["head"]["ref"] = "codex/setup-workflow"
        self.assertEqual(validate(self.pr), [])

    def test_reject_invalid_metadata(self):
        for key, value in [("title", "update"), ("head", {"ref": "main"}),
                           ("base", {"ref": "develop"})]:
            with self.subTest(key=key):
                pr = copy.deepcopy(self.pr)
                pr[key] = value
                self.assertTrue(validate(pr))

    def test_bot_exemption_is_exact(self):
        self.pr.update(title="Bump action", head={"ref": "dependabot/github_actions/action"})
        self.pr["user"]["login"] = "dependabot[bot]"
        self.assertEqual(validate(self.pr), [])
        self.pr["user"]["login"] = "dependabot-imposter"
        self.assertTrue(validate(self.pr))


if __name__ == "__main__":
    unittest.main()
