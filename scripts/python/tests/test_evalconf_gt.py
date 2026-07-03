import sys, os, numpy as np, subprocess
# synthetic: 1 fake dmap-array + GT where conf perfectly ranks -> AUC 1.0
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import EvalConfidence as E
def test_gt_label_metrics():
    d_gt  = np.full((10, 10), 5.0); d_gt[0, :] = np.nan
    d_est = np.full((10, 10), 5.0); d_est[5:, :] = 6.0     # bottom half = outliers
    conf  = np.where(d_est == 5.0, 0.9, 0.1)
    m = E.metrics_from_gt(d_est, conf, d_gt, rel_tol=0.01)
    assert m['roc_auc'] > 0.99 and m['n_labeled'] == 90
if __name__ == '__main__': test_gt_label_metrics(); print('OK')
