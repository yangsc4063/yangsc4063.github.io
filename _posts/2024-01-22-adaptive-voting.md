---
title: 投票法求解截断最小二乘优化
date: '2024-01-22 22:24:15 +0800'
categories: [Research]
tags: [pointcloud registraion, non-convex optimization]
math: true
---

# 投票法求解截断最小二乘优化

> 本文讨论求解目标函数为**截断凸函数和的优化问题的投票方法**。

为解决本文研究一类特殊的非凸优化问题，其目标函数可以写成截断凸函数之和，即

$$\begin{equation}\boldsymbol x=\arg\min_{\boldsymbol x}\sum_{i=1}^N\min\big(f_i(\boldsymbol x),\bar{\lambda}_i\big)\tag{1}\end{equation}$$

式中：$f_i：R^d→R,i=1,...,n$，为凸函数，$\bar{\lambda}_i∈R,i=1,...,n$，为常数。由于 $f_i(·)$ 在 $\bar{\lambda}_i$ 处的截断，目标函数往往是非凸的。

尽管一般来说，这类问题是 NP-hard 的，我们展示了在低维设置中，对于一些 $f_i(·)$ 存在一个全局极小值的多项式时间算法。

![image-20240123144811093](assets/img/20240122/image-20240123144811093.png)
_两个截断二次函数 $f_1,f_2$ 之和_

其思想很简单：当目标函数是分段凸的（如图），可以对定义域进行划分，使得在每个片段上限制目标函数为凸函数。通过这种方式，<u>枚举所有片段、在每个片段上最小化目标函数</u>，并在所有局部极小值中取最小值来找到全局极小值。

## 问题描述

考虑 $n$ 个凸函数 $f_i:R^d\rightarrow R$，其中 $i=1,...,n$，以及常数 $\bar{\lambda}_i∈R,i=1,...,n$。我们的目标是找到 $\boldsymbol x\in R^d$，使得 $(2)$ 在 $\boldsymbol x$ 处最小化：

$$\begin{equation}f(\boldsymbol x)=\sum_{i=1}^N\min\big(f_i(\boldsymbol x),\bar{\lambda}_i\big)\tag{2}\end{equation}$$

不失一般性，进一步假设对所有 i 都有 $\bar{\lambda}_i=0$，则最小化 $(1)$ 等价于最小化

$$\begin{equation}g(\boldsymbol x)=\sum_{i=1}^N\min\big(g_i(\boldsymbol x),0\big)+\sum_{i=1}^N\bar{\lambda}_i\tag{3}\end{equation}$$

其中，$g_i:R^d\rightarrow R$ 定义为 $g_i(x)=f_i(x)-\bar λ_i$，也是凸函数。更进一步，定义 $C_i\subset R^d$ 为 $g_i≤0$ 的凸区域。并且我们定义 $C_i$ 的边界 $\partial C_i:=\{x:g_i(x)=0\}$，作为 $f_i$ 的截断边界。那么，所有 $f_i$ 的截断边界 $\{∂C_i\}^N_{i=1}$，将区域 $R^d$ 划分为不相交的片 $A_1,...,A_M$，使得

$$\begin{equation}\begin{aligned}&A_j\cap A_k=\emptyset,j\neq k\\
&\bigcup_{i=1}^MA_i=R^d\end{aligned}\tag{4}\end{equation}$$

其中，

$$\begin{equation}A_j=\big(\bigcap_{k\in I_j}C_k\big)\cap\big(\bigcap_{l\notin I_j}C_l^c\big)\tag{5},I_j\subset\{1,...,N\},j=1,...,M\end{equation}$$

其中 $I_j$ 是 $\{ f_1,...,f_N \}$ 的指标集的一个子集，满足对任意的 $\boldsymbol x∈A_j$，若 $k∈I_j$，则 $g_k (\boldsymbol x)≤0$；若 $k\notin I_j$，则 $g_k (\boldsymbol x) > 0$。

![image-20240123172228898](assets/img/20240122/image-20240123172228898.png)
_定义在 $R^2$ 上的3个凸函数 $f_1,f_2,f_3$ 对应的 $C_i$_



## 一般算法

我们的目标是找到分区中每个区域 $A_j$ 上的局部极小值，并将所有局部极小值中的最小值作为全局解，即

$$\begin{equation}\min_{\boldsymbol x}\sum_{i=1}^N\min\{g_i(\boldsymbol x),0\}=\min_j\min_{\boldsymbol x\in A_j}\sum_{k\in I_j}g_k(\boldsymbol x)\tag{6}\end{equation}$$

当在 $A_j$ 范围内最小化 $\sum_{k∈I_j}f_k(x)$ 时，我们需要找到指标集合 $I_j$，并在 $\boldsymbol x ∈ A_j$ 的限制下。

> 尽管目标函数 $\sum_{k∈I_j}g_k(x)$ 是凸函数的和（因此也是凸函数），但域 $A_j$ 可以是非凸集合。如图，除了 $A_3$ 之外，所有其他 $A_j$ 都是非凸集合。
{: .prompt-warning }

解决这样的受限优化问题可能非常具有挑战性。不过，我们注意到，

一方面，

$$\begin{equation}\min_{\boldsymbol x}\sum_{i=1}^N\min\{g_i(\boldsymbol x),0\}=\min_j\min_{\boldsymbol x\in A_j}\sum_{k\in I_j}g_k(\boldsymbol x)\geq\min_j\min_{\boldsymbol x}\sum_{k\in I_j}g_k(\boldsymbol x)\tag{7}\end{equation}$$​

另一方面，

$$\begin{equation}\begin{aligned}\min_j\min_{\boldsymbol x}\sum_{k\in I_j}g_k(\boldsymbol x)&\geq\min_j\min_{\boldsymbol x}\sum_{k\in I_j}\min\{g_k(\boldsymbol x),0\}\\
&\geq\min_j\min_{\boldsymbol x}\sum_{i=1}^N\min\{g_k(\boldsymbol x),0\}\\
&=\min_{\boldsymbol x}\sum_{i=1}^N\min\{g_i(\boldsymbol x),0\}\end{aligned}\tag{8}\end{equation}$$

综合 $(7)$ 和 $(8)$ 可得，

$$\begin{equation}\min_{\boldsymbol x}\sum_{i=1}^N\min\{g_i(\boldsymbol x),0\}=\min_j\min_{\boldsymbol x}\sum_{k\in I_j}g_k(\boldsymbol x)\tag{9}\end{equation}$$

幸运的是，$(9)$ 表明在最小化 $\sum_{k∈I_j}g_k(\boldsymbol x)$ 时忽略约束 $\boldsymbol x∈A_j$ 是安全的，因此我们只需要求解一系列无约束的凸优化问题，这要容易得多。



## 截断最小二乘优化实现

上述通用算法的实现既依赖于函数类 $\{f_i\}^N_{i=1}$，又依赖于维数 $d$。

对于上述算法，当 $d = 1$ 时，每个 $C_i$ 是实线上的一个区间，$C_i$ 的边界 $∂C_i$ 由 $C_i$ 的两个端点组成，它们是 $g_i$ 穿过零的位置。不失一般性，假设 $\{∂C_i\}^N_{i=1}$ 的 $2N$ 个端点都是不同的，那么我们可以将它们沿实线顺序排列，将 $R$ 划分为 $M = 2N + 1$ 个片段 $\{A_j\}^M_{j=1}$。然后，我们可以一个接一个地顺序遍历它们，与此同时，计算每个片段 $A_j$ 上未截断函数之和。

### 目标函数的重新表述

对于鲁棒点云配准问题，其缩放估计和平移分量估计都是对未知标量的 TLS 优化。<u>以缩放估计为例</u>，通过上述算法和简单的枚举，可以在多项式时间内求解标量 TLS 优化。

对于缩放估计，

$$\begin{equation}\hat s=\arg\min_s\sum_{k=1}^K\min\bigg(\frac{(s-s_k)^2}{\alpha_k^2},\bar{c}^2\bigg)\tag{10}\end{equation}$$

$$\begin{equation}\begin{aligned}&f\Leftrightarrow\sum_{k=1}^K\min\bigg(\frac{(s-s_k)^2}{\alpha_k^2},\bar{c}^2\bigg)\\
&\{f_i\}^N_{i=1}\Leftrightarrow\bigg\{\frac{(s-s_k)^2}{\alpha_k^2}\bigg\}_{k=1}^K\\
&d=1\end{aligned}\tag{11}\end{equation}$$

于是，

$$\begin{equation}\begin{aligned}&g\Leftrightarrow\sum_{k=1}^K\min\bigg(\frac{(s-s_k)^2}{\alpha_k^2},0\bigg)+\sum_{k=1}^K\bar{c}^2\\
&\{g_i\}^N_{i=1}\Leftrightarrow\bigg\{\frac{(s-s_k)^2}{\alpha_k^2}-\bar{c}^2\bigg\}_{k=1}^K\\
&C_i=[s_k-\alpha_k\bar{c},s_k+\alpha_k\bar{c}]\\
&\partial C_i=\big\{s_k-\alpha_k\bar{c},s_k+\alpha_k\bar{c}\big\}_{k=1}^K\end{aligned}\tag{12}\end{equation}\\$$

对 $(12)$ 中 $\partial C_i$，使其沿实线顺序排列，将 $R$ 划分为 $2K+1$ 个片段 $\lbrace A_j\rbrace^{2K+1}\_{j=1}$。同理，我们需要 $I_j$，$\{1,...,K\}$ 的一个子集，满足对任意的 $s∈A_j$，若 $k∈I_j$，则 $(s-s_k)^2-\alpha_k^2\bar{c}^2≤0$；若 $k\notin I_j$，则 $(s-s_k)^2-\alpha_k^2\bar{c}^2>0$。容易想到，对于小于最小边界的片段和大于最大边界的片段，对应的 $I_j$ 是平凡的（必为空集），因此，可 $\lbrace A_j\rbrace^{2K+1}\_{j=1}$ 重新编号，即仅考虑除去除上述两片段之外的 $2K-1$ 个片段，得到  $\lbrace A_j\rbrace^{2K-1}\_{j=1}$，其对应的 $I_j$ 分别为：$I_1,I_2,...,I\_{2K-1}$。

那么，原本被 $(10)$ 表述的缩放估计问题，可被重新表述为，求解 $(13)$ 取得时所对应的 $s$：

$$\begin{equation}\min_j\min_{s}\sum_{k\in I_j}\bigg[\frac{(s-s_k)^2}{\alpha_k^2}-\bar{c}^2\bigg]\tag{13}\end{equation}$$

其中，$\min_{s}\sum\_{k\in I_j}\big[\frac{(s-s_k)^2}{\alpha_k^2}-\bar{c}^2\big]$ 容易求解，$\hat{s}_j=\big(\sum\_{k\in I_j}\frac{1}{\alpha_j^2}\big)^{-1}\sum\_{k\in I_j}\frac{s_k}{\alpha_j^2}$，我们只需从中枚举出最小的作为 $\hat s$ 即可。

### 投票法求解

由上文可知，求解的关键在于统计 $A_j$ 对应的 $I_j$ 中的元素，使用投票法进行求解，其原理如下：

![image-20240124012400022](assets/img/20240122/image-20240124012400022.png)

如图观察可知，<u>任意一个 $A_j$ 均为 $C_j$ 的真子集，因此，我们计算每个 $A_j$ 的中点，并判断其是否落在 $C_j$ 内部即可。</u>其伪代码如下：

![image-20240124012715812](assets/img/20240122/image-20240124012715812.png)

> 至此，我们通过自适应投票法在多项式时间内精确求解了标量 TLS 问题，这对于缩放估计和平移分量估计都是有效的。甚至在 2D 点云配准问题中，由于旋转仅一个自由度，也能够使用此方法。该方法优点在于能够比较好得考虑点云测量噪声的影响，且允许我们对点云的噪声模型进行更精细的建模。
{: .prompt-tip }
