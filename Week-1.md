# Preparation

## Some useful materials

### Bayesian Optimisation (BO) Introduction

*BO is a way to compute global optima.*

*The 1-minute summary is at:*

*https://www.youtube.com/watch?v=WkZueBgKFYM&ab_channel=YukiKoyama*

*A slightly more detailed summary is at:*

[*https://www.youtube.com/watch?v=41gKFmKQDlg&t=381s&ab_channel=MacCormac*](https://www.youtube.com/watch?v=41gKFmKQDlg&t=381s&ab_channel=MacCormac)

### Problem Intro

The problem we are interested in is **being able to choose the information matrix in a factor graph**. This problem appears to have not been widely tackled, and I haven't been able to find much in the way of papers on the topic. However, a related problem is in Kalman filtering. The problem there is called tuning and is about picking the Q and R matrices (process and observation noise covariances).

### Kalman Filter Quick-reminder

From having a quick look, a good introduction is in lectures 4.3.2 and 4.3.3 of this lecture series:

[https://www.youtube.com/watch?v=UmC2OaJWBeI&list=PLTD_k0sZVYFqjFDkJV8GE2EwfxNK59fJY&ab_channel=LarsHammarstrand](https://eur01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fwww.youtube.com%2Fwatch%3Fv%3DUmC2OaJWBeI%26list%3DPLTD_k0sZVYFqjFDkJV8GE2EwfxNK59fJY%26ab_channel%3DLarsHammarstrand&data=05|01|zuxun.wang.22%40ucl.ac.uk|a0649f57edbc4d0d680a08db466324a7|1faf88fea9984c5b93c9210a11d9a5c2|0|0|638181163843520324|Unknown|TWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D|3000|||&sdata=U3IC5nhxjIpXFmRovzZkH%2FmGfrbgNcuJpaTwxSNjxv0%3D&reserved=0)

In particular, in 4.3.2 you want to look from about 05:32 onwards. (The comment is correct - the specified disribution is wrong. However, the first two moments are correct, and this fact will potentially be useful later in your project.)

### BO for filtering

A basic paper on the Bayes optimization approach for filtering can be found at:

https://arxiv.org/abs/1807.08855v1

We have a paper which is being developed which extends this.

For factor graphs, the crucial result we need is from this paper:

[https://www.mit.edu/~mrrobot/assets/khosoussi19ijrr.pdf](https://eur01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fwww.mit.edu%2F~mrrobot%2Fassets%2Fkhosoussi19ijrr.pdf&data=05|01|zuxun.wang.22%40ucl.ac.uk|a0649f57edbc4d0d680a08db466324a7|1faf88fea9984c5b93c9210a11d9a5c2|0|0|638181163843520324|Unknown|TWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D|3000|||&sdata=sPXyeAHptU2H8eUwu4MgDRXKvdGP02SsrqnmBy9ZsbA%3D&reserved=0)

The main thing we need is proposition 4 - this tells us that the residuals in a well-tuned factor graph should look a bit like the $$chi^2$$ distribution we get with the tuning for the Kalman filter. Given that, it suggests we could use the Kalman filter tuning method for the factor graph.

## Summary of matrials

![image-20230507212435870](/Users/wangzuxun/Library/Application Support/typora-user-images/image-20230507212435870.png)

![image-20230507212545903](/Users/wangzuxun/Library/Application Support/typora-user-images/image-20230507212545903.png)

![image-20230507221431020](/Users/wangzuxun/Library/Application Support/typora-user-images/image-20230507221431020.png)

## Meeting

for my project, I only have the sensor data.

Based on the pervious work, and develop your own stuff.

Basically, read both two papers as they are equally important.

proposition 3 4 on the paper.

	- try to prove proposition 3? 
	- to elaborate on different cases, where 3 have ground truth for initialisation, you have some specific chi-2 values.
	- 4 does not, but also result in some chi-2 value different to 3, but noisy.

workshop2 - code base from topic 2.

​	- maybe try to search about different estimation algorithms

Fill in the Project Plan form.

​	- Ethics. Nothing really we need to worry too much as we don't apply robots on street which may invovle privacy issues.

early june -> introduction.

---

Simon also provide a short template for each meeting to remind himself what has been discussed and planned for the next week.