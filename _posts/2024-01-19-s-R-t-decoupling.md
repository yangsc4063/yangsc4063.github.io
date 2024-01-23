---
title: 缩放、旋转和平移解耦的点云配准
date: 2024-01-08 20:38:29 +0800
categories: [Research]
tags: [pointcloud registraion]
math: true
---

# 缩放、旋转和平移解耦的点云配准

> 本文是对论文《TEASER: Fast and Certifiable Point Cloud Registration》中所提出的**缩放、旋转和平移解耦的点云配准**的梳理。

在鲁棒点云配准问题中，给定两个 3D 点云 $\boldsymbol P=\lbrace\boldsymbol p_i\rbrace\_{i=1}^N$ 和 $\boldsymbol Q=\lbrace\boldsymbol q_i\rbrace\_{i=1}^N$，其中 $\boldsymbol p_i,\boldsymbol q_i\in R^3$。考虑基于对应的模型，即假设给定的对应 $(\boldsymbol p_i,\boldsymbol q_i),i=1,...,N$服从以下模型：

$$\begin{equation}\boldsymbol q_i=s^\circ\boldsymbol R^\circ\boldsymbol p_i+\boldsymbol t^\circ+\boldsymbol o_i+\boldsymbol \varepsilon_i\tag{1}\end{equation}$$

对模型 $(1)$ 进行重新表述，以获得对部分变换（缩放、旋转、平移）不变的量。

## 不变度量

### 平移不变度量（TIMs）

虽然 $Q$ 中的点的绝对位置取决于平移 $\boldsymbol t$，但相对位置对 $\boldsymbol t$ 是不变的。在数学上，给定来自 $(1)$ 的两个点 $\boldsymbol q_i$ 和 $\boldsymbol q_j$，则这两点的相对位置：

$$\begin{equation}\boldsymbol q_j-\boldsymbol q_i=s^\circ\boldsymbol R^\circ(\boldsymbol p_j-\boldsymbol p_i)+(\boldsymbol o_j-\boldsymbol o_i)+(\boldsymbol \varepsilon_j-\boldsymbol \varepsilon_i)\tag{2}\end{equation}$$

![image-20240121164943879](assets/img/20240119/image-20240121164943879.png)
_由Bunny数据集中的完整图生成的TIMs_

其中，平移 $\boldsymbol t$ 在减法中被消除。因此，我们可以通过计算 $\boldsymbol p_{ij}=\boldsymbol p_j-\boldsymbol p_i$ 和 $\boldsymbol q_{ij}=\boldsymbol q_j-\boldsymbol q_i$ 来获得平移不变度量（TIM），而 TIM 满足以下模型：

$$\begin{equation}\boldsymbol q_{ij}=s^\circ\boldsymbol R^\circ\boldsymbol p_{ij}+\boldsymbol o_{ij}+\boldsymbol \varepsilon_{ij}\tag{3}\end{equation}$$

其中，$\boldsymbol o_{ij}$ 在第 i 和第 j 个点都是<u>内点</u>时为零（或在其他情况下是任意的），而 $\boldsymbol \varepsilon_{ij}$ 是测量噪声。很容易看出，如果 $\|\boldsymbol\varepsilon_i\|\leq\beta_i,\|\boldsymbol\varepsilon_j\|\leq\beta_j$，则 $\|\boldsymbol\varepsilon_{ij}\|\leq\beta_i+\beta_j=\delta_{ij}$。

TIMs 在式 $(3)$ 中的优势在于其模型<u>只取决于两个未知数 $s$ 和 $\boldsymbol R$</u>。TIMs 的数量上限为 $N(N-1)/2$，其中计算了所有点对之间的相对测量。

### 平移和旋转不变度量（TRIMs）

虽然点对的相对位置（TIMs）仍然依赖于旋转 $\boldsymbol R$，但它们的距离对 $\boldsymbol R$ 和 $\boldsymbol t$ 都是不变的。因此，为了建立旋转不变的测量，我们计算每个 TIM 向量的范数：

$$\begin{equation}\|\boldsymbol q_{ij}\|=\|s^\circ\boldsymbol R^\circ\boldsymbol p_{ij}+\boldsymbol o_{ij}+\boldsymbol \varepsilon_{ij}\|\tag{4}\end{equation}$$

对于内点（$\boldsymbol o_{ij}=\boldsymbol 0$），利用 $\|\boldsymbol\varepsilon_{ij}\|=\delta_{ij}$ 和三角不等式，有：

$$\begin{equation}\|s^\circ\boldsymbol R^\circ\boldsymbol p_{ij}\|-\delta_{ij}\leq\|s^\circ\boldsymbol R^\circ\boldsymbol p_{ij}+\boldsymbol \varepsilon_{ij}\|\leq\|s^\circ\boldsymbol R^\circ\boldsymbol p_{ij}\|+\delta_{ij}\tag{5}\end{equation}$$

可以把 $(4)$ 等价地写成：

$$\begin{equation}\|\boldsymbol q_{ij}\|=\|s^\circ\boldsymbol R^\circ\boldsymbol p_{ij}\|+\widetilde{o}_{ij}+\widetilde{\varepsilon}_{ij}\tag{6}\end{equation}$$

其中 $\widetilde{\varepsilon}\_{ij}\leq\delta\_{ij}$，如果第 i 和第 j 个点都是内点，$\widetilde{o}\_{ij}=0$，否则为任意标量。范数是旋转不变的，并且 $s>0$，将等式 $(6)$ 两边除以 $\|\boldsymbol p\_{ij}\|$，我们得到新的测量 $s\_{ij}=\|\boldsymbol q\_{ij}\|/\|\boldsymbol p\_{ij}\|$。

$$\begin{equation}s_{ij}=s+o_{ij}^s+\varepsilon_{ij}^s\tag{7}\end{equation}$$

其中，$\varepsilon\_{ij}^s=\widetilde{\varepsilon}\_{ij}/\|\boldsymbol p\_{ij}\|$，$o\_{ij}^s=\widetilde{o}\_{ij}/\|\boldsymbol p\_{ij}\|$。由于 $\|\widetilde\varepsilon\_{ij}\|\leq\delta\_{ij}$，很容易看出 $\|\varepsilon\_{ij}^s\|\leq\delta\_{ij}/\|\boldsymbol p\_{ij}\|$。我们定义 $α\_{ij}=δ\_{ij}/\|\boldsymbol p\_{ij}\|$。

式 $(7)$ 描述了一种平移和旋转不变测量（TRIM），<u>其模型仅是未知尺度 $s$ 的函数</u>。

### 小结

|  度量  |                              点                              |                             TIMs                             |                            TRIMs                             |
| :----: | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
|  符号  |              $\boldsymbol p_i,\boldsymbol q_i$               |           $\boldsymbol p_{ij},\boldsymbol q_{ij}$            |                           $s_{ij}$                           |
|  定义  |                              -                               | $\begin{cases}\boldsymbol p_{ij}=\boldsymbol p_j-\boldsymbol p_i\\\boldsymbol q_{ij}=\boldsymbol q_j-\boldsymbol q_i\end{cases}$ | $s_{ij}=\frac{\|\boldsymbol q_{ij}\|}{\|\boldsymbol p_{ij}\|}$ |
|  模型  | $\boldsymbol q_i=s^\circ\boldsymbol R^\circ\boldsymbol p_i+\boldsymbol t^\circ+\boldsymbol o_i+\boldsymbol \varepsilon_i$ | $\boldsymbol q_{ij}=s^\circ\boldsymbol R^\circ\boldsymbol p_{ij}+\boldsymbol o_{ij}+\boldsymbol \varepsilon_{ij}$ |            $s_{ij}=s+o_{ij}^s+\varepsilon_{ij}^s$            |
| 噪声界 |          $\|\boldsymbol \varepsilon_i\|\leq\beta_i$          | $\|\boldsymbol\varepsilon_{ij}\|\leq\delta_{ij}=\beta_i+\beta_j$ | $\|\varepsilon_{ij}^s\|\leqα_{ij}=\delta_{ij}/\|\boldsymbol p_{ij}\|$ |
|  数量  |                             $N$                              |                   $K\leq\frac{N(N-1)}{2}$                    |                             $K$                              |

## 缩放、旋转和平移级联求解

### 缩放估计

TRIM 模型描述了未知尺度 $s$ 的线性标量度量 $s_{ij}$，它受到有界噪声 $\|\varepsilon_{ij}^s\|\leqα_{ij}=\delta_{ij}/\|\boldsymbol p_{ij}\|$ 的影响，其中可能包括潜在的异常值（当 $o_{ij}^s\neq0$ 时）。给定度量 $s_{ij}$ 和边界 $\alpha_{ij}$，使用截断最小二乘（TLS）来估计缩放：

$$\begin{equation}\hat s=\arg\min_s\sum_{k=1}^K\min\bigg(\frac{(s-s_k)^2}{\alpha_k^2},\bar{c}^2\bigg)\tag{8}\end{equation}$$

其中，为了简单起见，我们将不变量度量从 $1$ 编号到 $K$，并采用符号 $s_k$ 代替 $s_{ij}$。

### 旋转估计

给定缩放估计 $\hat{s}$，TIM 模型描述了受到有界噪声 $\|\boldsymbol\varepsilon_k\|\leq\delta_k$，其中可能包括潜在的异常值（当 $\boldsymbol o_k\neq0$ 时）影响的度量 $\boldsymbol q_k$。使用截断最小二乘法（TLS）从估计的尺度 $\hat{s}$、TIM 度量（$\boldsymbol p_k,\boldsymbol q_k$）和边界 $\delta_k$ 中，估计旋转 $\boldsymbol R$：

$$\begin{equation}\hat{\boldsymbol R}=\arg\min_{\boldsymbol R\in SO(3)}\sum_{k=1}^K\min\bigg(\frac{\|\boldsymbol q_k-\hat{s}\boldsymbol p_k\|}{\delta_k^2},\bar{c}^2\bigg)\tag{9}\end{equation}$$

### 平移估计

在通过求解 $(8)$ 和 $(9)$ 获得尺度和旋转估计 $\hat s$ 和 $\hat{\boldsymbol R}$ 后，我们可以将它们代入

$$\begin{equation}\min_{s>0,\boldsymbol R\in SO(3),\boldsymbol t\in R^3}\sum_{i=1}^N\frac{1}{\sigma_i^2}\|\boldsymbol q_i-s\boldsymbol R\boldsymbol p_i-\boldsymbol t\|^2\tag{10}\end{equation}$$

中以估计平移 $\boldsymbol t$。尽管 $(10)$ 对矢量的 2-范数进行操作，仍建议独立地计算 $\boldsymbol t$ 的分量，即：

$$\begin{equation}\hat t_j=\arg\min_{t_j}\sum_{k=1}^K\min\bigg(\frac{(t_j-[\boldsymbol q_i-\hat s\hat{\boldsymbol R}\boldsymbol p_i]_j)^2}{\beta_j^2},\bar{c}^2\bigg)\tag{11}\end{equation}$$

其中 $j=1,2,3$，其中 $[·]_j$ 表示向量的第 $j$ 个条目。由于 $\boldsymbol q_i-\hat s\hat{\boldsymbol R}\boldsymbol p_i$ 在此阶段是一个已知的向量，很容易看出 $(11)$ 是一个标量 TLS 问题。因此，其求解方式类似式 $(8)$。

> 如何求解 $(9),(10),(11)$ 目标函数高度非凸且计算复杂度 $O(2^N)$ 的优化问题，是解决鲁棒点云配准问题的关键。
{: .prompt-tip }
