
from math import *


def func(mu, sigma2, x):
    return 1/sqrt(2.*pi*sigma2) * exp(-.5 * (x - mu)**2 / sigma2)


print(func(10., 4., 8.))
