import numpy as np

# split x and y into random batches
# return a list of [(batch1_x,batch1_y)...]
def get_random_batches(x,y,batch_size):
    batches = []
    batch_cnt = x.shape[0] // batch_size
    if x.shape[0] % batch_size != 0:
        batch_cnt += 1

    vals = np.arange(0,x.shape[0])
    for i in range(batch_cnt-1):
        batch_idx = np.random.choice(vals, batch_size, replace=False)

        # remove elems that have been selected
        for j in range(len(batch_idx)):
            posDex = np.where(vals==batch_idx[j])[0][0]
            vals = np.delete(vals, posDex)

        if len(x.shape) == 1:
            batches.append( (x[batch_idx], y[batch_idx]) )
        else:
            batches.append( (x[batch_idx,:], y[batch_idx]) )

    # last batch often is less than batch size, we can still include that data
    # by doing this though:
    if len(x.shape) == 1:
        batches.append( (x[vals], y[vals]) )
    else:
        batches.append( (x[vals,:], y[vals]) )

    return batches


def compute_loss_and_acc(y_actual, y_predicted, acceptable_diff=25):
    """
    Parameters:
    acceptable_diff - [float] range (-diff, diff) that NN estimate needs to be
                        in to be deemed 'correct'
    """
    loss, acc = None, None

    y_actual.flatten()
    y_predicted.flatten()

    loss = np.sum( (y_actual - y_predicted)**2 ) / y_actual.shape[0] # MSE loss

    acc = np.sum( (np.abs(y_actual - y_predicted) < acceptable_diff).astype(np.int) ) / y_actual.shape[0]

    return loss, acc
