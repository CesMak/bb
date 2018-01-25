from matplotlib import pyplot as plt
import numpy as np
from Pend2dBallThrowDMP import *
%matplotlib inline

env = Pend2dBallThrowDMP()

numDim = 10
numSamples = 25
maxIter = 100
numTrials = 10

# Do your learning


# For example, let initialize the distribution...
Mu_w = np.zeros(numDim)
Sigma_w = np.eye(numDim) * 1e6

# ... then draw a sample and simulate an episode
sample = np.random.multivariate_normal(Mu_w,Sigma_w)
reward = env.getReward(sample)

# Save animation
env.animate_fig ( np.random.multivariate_normal(Mu_w,Sigma_w) )
plt.savefig('EM-Ex2.pdf')
