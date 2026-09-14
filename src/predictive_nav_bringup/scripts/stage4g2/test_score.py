#!/usr/bin/env python3
"""Checks the evaluation gate, one-to-one scoring and physical merge diagnosis."""
import unittest

from score import assignment, visibility


class EvaluationTests(unittest.TestCase):
    def test_gate_and_cardinality(self):
        # Greedy would consume the only eligible detection of GT B.
        # Evaluation matching must preserve maximum cardinality.
        self.assertEqual(assignment([[0.,0.],[.4,0.]],[[.1,0.],[-.3,0.]],.45),{0:1,1:0})
        self.assertEqual(assignment([[0.,0.]],[[1.,0.]],.45),{})
        self.assertEqual(assignment([],[[0.,0.]]),{})
        self.assertEqual(len(assignment([[0.,0.],[0.,.01]],[[0.,0.]])),1)

    def test_visibility_is_not_gt_presence(self):
        # Actual returns only from the near cylinder: second GT exists but is unobserved.
        scan={'points':[[-.2,0.],[-.199,.02],[-.199,-.02]]}
        counts,merges=visibility(scan,[[0.,0.],[1.,0.]])
        self.assertEqual(counts,[3,0])
        self.assertEqual(merges,[])

    def test_merge_requires_returns_from_both_objects(self):
        scan={'points':[[-.2,0.],[-.199,.02],[-.199,-.02],
                        [-.2,.30],[-.199,.32],[-.199,.28]]}
        counts,merges=visibility(scan,[[0.,0.],[0.,.30]])
        self.assertEqual(counts,[3,3])
        self.assertEqual(len(merges),1)
        self.assertEqual(set(merges[0]),{0,1})


if __name__=='__main__':
    unittest.main()
