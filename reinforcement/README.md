# CS188 Berkeley, Artificial Intelligence
## Reinforcement Learning Project

## 0 Basic
- **environment**: python2.7
- **commands**:
```
  # run gridworld in manual control
  python gridworld.py -m
  # help list
  python gridworld.py -h
  # random moving agent
  python gridworld.py -g MazeGrid
```

## 1 Value Iteration
In this practice, we need to implement a Module-based MDP solver. Since we have already know the transitions $T(s,a,s^\prime)$ and the rewards $R(s, a, s^\prime)$, we can use Bellman equation recursively and calculate the Q_value for each (state, action) pair. Then use the max Q-value in each state to find a best solution.
$$V^*(s) = \max_a Q^*(s,a)$$
$$Q^*(s,a) = \sum_{s^\prime} T(s,a,s^\prime) \left[ R(s,a,s^\prime) + \gamma V^*(s^\prime)\right]$$
$$V^*(s) = \max_a \sum_{s^\prime}T(s,a,s^\prime)\left[ R(s,a,s^\prime)+\gamma V^*(s^\prime)\right]$$
```
  # check the result
  python gridworld.py -a value -i 5
```

## 2 Q-learning
The Q-learning is a Model-free method to solve MDP. This means the transitions $T(s,a,s^\prime)$ is unknown. In this case, we won't try to calculate the transitions $T$, and we will learn by trial.
- At the very beginning, $Q_0(s,a) = 0$
- receive a sample $(s,a,s^\prime, r)$
- find out the old estimate: $Q(s,a)$
- calculate the sample, $\epsilon$ is a discount rate:
    $$sample = R(s,a,s^\prime) + \gamma \max_{a^\prime}Q(s^\prime, a^\prime)$$
- update $Q(s,a)$, $\alpha$ is a learning rate:
  $$Q(s,a) \leftarrow (1-\alpha)Q(s,a) + (\alpha)[sample]$$
```
  # check the result
  python gridworld.py -a q -k 5 -m
```

## 3 epsilon-greedy
It is a Simple method to explore the search space. When we use Model-Based method to solve MDP, we always take the largest Q-value to choose next action, which is a greedy strategy. But in Model-free method, it may not work when we training the MDP. Thus, we need a method to explore the search space. $\epsilon$-greedy is a simple method to do so.
- each step, generate a random number.
- with probability $\epsilon$, select a legal action randomly from the legal action list.
- with probability $(1-\epsilon)$, select the action with max Q-value.
```
  # a funny crawler learning demo
  python crawler.py
```

## 4 Approximate Q-learning
We can use a linear function to make $Q(s,a)$ more informatic.
- fetch some features from your problem, and put them in a linear function($f_i(s,a)$ stands for a feature, such as counting the number of people still in the room. $w_i$ is its weight in this MDP. We will learn this):
$$Q(s,a) = \sum\limits_{i=1}^n f_i(s,a) w_i$$
- Then update all the $w_i$ when we get a sample.
$$w_i \leftarrow w_i + \alpha \cdot difference \cdot f_i(s,a)$$
$$difference = (r + \gamma \max\limits_{a'} Q(s', a')) - Q(s,a)$$
```
  # pacman...
  python pacman.py -p ApproximateQAgent -a extractor=SimpleExtractor -x 50 -n 60 -l mediumGrid
```
