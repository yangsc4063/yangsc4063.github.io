---
title: 鲁棒点云配准问题
date: 2024-01-08 20:38:29 +0800
categories: [Research]
tags: [pointcloud registraion]
math: true
---

# 鲁棒点云配准问题

> 本文是对论文《TEASER: Fast and Certifiable Point Cloud Registration》中所提出的**鲁棒点云配准问题**的梳理。

在鲁棒点云配准问题中，给定两个 3D 点云 $\boldsymbol P=\lbrace\boldsymbol p_i\rbrace\_{i=1}^N$ 和 $\boldsymbol Q=\lbrace\boldsymbol q_i\rbrace\_{i=1}^N$，其中 $\boldsymbol p_i,\boldsymbol q_i\in R^3$。考虑基于对应的模型，即假设给定的对应 $(\boldsymbol p_i,\boldsymbol q_i),i=1,...,N$服从以下模型：

$$\boldsymbol q_i=s^\circ\boldsymbol R^\circ\boldsymbol p_i+\boldsymbol t^\circ+\boldsymbol o_i+\boldsymbol \varepsilon_i$$

其中 $s^\circ>0$，$\boldsymbol R^\circ\in SO(3)$ 和 $\boldsymbol t^\circ\in R^3$ 是未知的（待计算的）尺度、旋转和平移真值，$\boldsymbol \varepsilon_i$ 是测量噪声的模型。如果点对 $(\boldsymbol p_i,\boldsymbol q_i)$ 是内点（正常数据，可被模型描述的数据），$\boldsymbol o_i$ 是零向量，如果点对 $(\boldsymbol p_i,\boldsymbol q_i)$ 是离群点（异常数据，无法适应模型的数据），$\boldsymbol o_i$ 是任意向量。换句话说，如果第 $i$ 个点对 $(\boldsymbol p_i,\boldsymbol q_i)$ 是内点对应，则 $\boldsymbol q_i$ 对应 $\boldsymbol p_i$（加噪声 $\boldsymbol \varepsilon_i$）的一个 3D 变换，而如果 $(\boldsymbol p_i,\boldsymbol q_i)$ 是离群点对应，则 $\boldsymbol q_i$ 只是一个任意的向量。

## 无离群点的点云配准

当 $\boldsymbol \varepsilon_i$ 是具有各向同性协方差 $\sigma_i^2\boldsymbol I_3$ 的零均值高斯噪声，且所有对应都是正确的（即 $\boldsymbol o_i=0,\forall i$）时，则$(s^\circ,\boldsymbol R^\circ,\boldsymbol t^\circ)$的最大似然估计可以通过求解如下非线性最小二乘问题来计算：

$$\min_{s>0,\boldsymbol R\in SO(3),\boldsymbol t\in R^3}\sum_{i=1}^N\frac{1}{\sigma_i^2}\|\boldsymbol q_i-s\boldsymbol R\boldsymbol p_i+\boldsymbol t\|^2$$

虽然式 $\eqref{2}$ 是一个非凸优化问题，但由于集合 $SO(3)$ 的非凸性，其最优解可以通过解耦尺度、旋转和平移估计，使用 SVD 分解以闭式计算（即可求解析解）。

在实际应用中，由于点匹配不正确，很大一部分对应是离群点。尽管闭式解形式优美，但它们对异常值并不鲁棒，即使单独一个"坏"的异常值就可能对估计的正确性造成负面影响。因此，我们提出了一种能够容忍极端数量离群对应的截断最小二乘配准公式。

## 截断最小二乘的点云配准

从高斯噪声模型出发，假设噪声是未知但有界的。形式上，我们假设内点噪声 $\boldsymbol \varepsilon_i$ 满足 $‖\boldsymbol \varepsilon_i‖\leq\beta_i$，其中 $\beta_i$ 是一个给定的边界。

然后我们采用以下截断最小二乘（Truncated Least Squares，TLS）配准公式：

$$\min_{s>0,\boldsymbol R\in SO(3),\boldsymbol t\in R^3}\sum_{i=1}^N\min\bigg(\frac{1}{\beta_i^2}\|\boldsymbol q_i-s\boldsymbol R\boldsymbol p_i+\boldsymbol t\|^2,\bar{c}^2\bigg)$$

该方法计算具有小残差的测量的最小二乘解 $\big(\frac{1}{\beta_i^2}\|\boldsymbol q_i-s\boldsymbol R\boldsymbol p_i+\boldsymbol t\|^2\leq\bar{c}^2\big)$，同时丢弃具有大残差的测量（当 $\frac{1}{\beta_i^2}\|\boldsymbol q_i-s\boldsymbol R\boldsymbol p_i+\boldsymbol t\|^2>\bar{c}^2$ 时，第 $i$ 个求和项变为常数，不影响优化）。

实际上，噪声边界 $\beta_i$ 在算法实现中相当容易设置，可以理解为 “3-sigma” 噪声边界，或者是我们所期望的内点中的最大误差。

### $\beta_i$ 与 $\bar{c}$ 的选择

<u>内点噪声的概率模型：</u>

如果我们假设内点服从模型 $\boldsymbol q_i=s^\circ\boldsymbol R^\circ\boldsymbol p_i+\boldsymbol t^\circ+\boldsymbol o_i+\boldsymbol \varepsilon_i$，且满足 $\boldsymbol\varepsilon_i\sim N(\boldsymbol0_3,\sigma_i^2\boldsymbol I_3)$，$\boldsymbol o_i=\boldsymbol 0$，则有

$$\frac{1}{\sigma_i^2}\|\boldsymbol q_i-\boldsymbol R\boldsymbol p_i\|^2=\frac{1}{\sigma_i^2}\|\boldsymbol \varepsilon_i\|^2\sim\chi^2(3),$$

其中，$\chi^2(3)$ 为具有三个自由度的卡方分布。因此，在期望的概率 $p$ 下，内点的加权误差 $\frac{1}{\sigma_i^2}\|\boldsymbol \varepsilon_i\|^2$ 满足：

$$P\bigg(\frac{1}{\sigma_i^2}\|\boldsymbol \varepsilon_i\|^2\leq\gamma^2\bigg)=p,$$

其中 $γ^2$ 是具有三个自由度的 $χ^2$ 分布的分位数，其左尾概率等于 $p$（例如，对于 $p = 0.97$，$γ=3$）。因此，可以简单地将噪声边界 $β_i$ 设置为 $β_i= σ_i$，令 $\bar c= γ$。参数 $γ$ 随着 $p$ 的增大而单调增加，因此，将 $p$ 设定为接近 $1$ 可使得公式 $\eqref{3}$ 更容易接受具有较大残差的测量，而较小的 $p$ 会使式 $\eqref{3}$ 更具选择性。

> 虽然我们假设对于内点我们有最大误差的边界 $\beta_i$，但我们不对离群点模型做任何假设，因为这在实践中往往是未知的。
> {: .prompt-tip }

### 截断最小二乘估计与一致性优化

此外，截断最小二乘估计与<u>一致性优化</u>（Consensus Maximization，MC）相关，MC 是计算机视觉中一种流行的鲁棒估计方法。MC 寻求一个能够最大化内点数量的估计，而 TLS 同时计算内点的最小二乘估计。总的来说，这两种方法一般不能保证产生相同的内点选择，因为 TLS 还会对具有较大误差的内点进行惩罚。

一致性最大化寻求一个能够最大化内点数量的估计，等价地，最小化离群点的数量的估计：

$$\min_{\mathcal{O}\subseteq\mathcal{M},\boldsymbol x\in\mathcal{X}}|\mathcal{O}|,\ s.t.\|r_i(\boldsymbol x_i)\|^2\leq\bar{c}^2\ \forall i\in\mathcal{M}\setminus\mathcal{O}$$

其中，$\boldsymbol x$ 是我们要估计（可能属于某个区域 $\mathcal X$）的变量，$\mathcal M$ 是可用的测量值集合，$r_i (·)$ 是给定的残差函数，$\bar c$ 是内点的最大容许误差，$|·|$表示集合的基数（集合元素的个数）。式 $\eqref{6}$ 寻找最小的离群点集（$\mathcal O$），使得对某个 $\boldsymbol x$，其它所有的测量值（即，内点 $\mathcal{M}\setminus\mathcal{O}$）都有小于 $\bar c$ 的残差。虽然一致性最大化问题在一般情况下是难以解决的，但通过自适应投票法，在标量情况下，它可以在多项式时间内求解。

一般来说，MC 和 TLS 不会返回相同的解，因为 TLS 可能更倾向于丢弃导致估计产生较大偏差的测量值，如下面的简单例子所示。

> 例：考虑一个简单的标量估计问题，其中我们给出了三个测量值，并比较两个公式：
>
> TLS：
>
> $$\min_s\sum_{k\in\mathcal{M}}\min\big((s-s_k)^2,\bar{c}^2\big),$$
>
> MC：
>
> $$\min_{\mathcal{O}\subseteq\mathcal{M},\boldsymbol s}|\mathcal{O}|,\ s.t.(s-s_k)^2\leq\bar{c}^2\ \forall k\in\mathcal{M}\setminus\mathcal{O}$$
>
> 其中 $\mathcal M=\{1,2,3\}$ 。设 $s_1=s_2=0,s_3=3$，$\bar{c}=2$。
>
> 可以看到 $s_{MC}=1.5$，且所得最大一致集包含了所有测量 $\{1,2,3\}$，而 $s_{TLS}=0$，且代价等于 $2$，一致集仅包含 $\{1,2\}$。

<u>虽然 MC 和 TLS 一般不会选择相同的内点集，但在内点集基数较大的情况下，我们期望 TLS 和 MC 产生类似的解决方案。</u>

**TLS 和 MC 等价的必要条件**：假设 MC 所求的最大一致集的大小为 $N^{MC}\_{in}$，内点的残差平方和为 $r^{MC}\_{in}$。如果次大一致性集合的规模小于 $N^{MC}\_{in}-r^{MC}\_{in}/\bar{c}^2$，则 TLS 也会选择该最大一致集作为内点。

证明：

对 TLS 公式，

$$\min_{\boldsymbol x\in\mathcal{X}}\sum_{i\in\mathcal{M}}\min\big(\|r_i(\boldsymbol x)\|^2,\bar{c}^2\big),$$

内点是在最优解处残差小于 $\bar{c}$ 的测量值。我们记 $f_{TLS}(\mathcal I)$ 为给定的一致集 $\mathcal I$ 的 TLS 代价的值。

记测量集的大小为 $N$，即 $N=\vert\mathcal M\vert$。如果 $\mathcal O\_{MC}$ 是 $\eqref{6}$ 的最优解，我们定义 $N^{MC}\_{out}=\vert\mathcal O\_{MC}\vert$ 。则通过 MC 找到的内点个数为 $N^{MC}\_{in}=\vert\mathcal M\setminus\mathcal O\vert=N-N^{MC}\_{out}$。此外，内点的残差平方和为：

$$r_{in}^{MC}=\sum_{i\in\mathcal M\setminus\mathcal O_{MC}}\|r_i(\boldsymbol x)\|^2,$$

反证，假设最大一致集 $\mathcal I_{MC}$ 导致次优的 TLS 解。那么，存在另一个 TLS 解，其一致集为 $\mathcal I'$，满足：

$$f_{TLS}(\mathcal I')<N^{MC}_{out}\bar{c}^2+r_{in}^{MC}$$

这是因为我们假设与 $\mathcal I_{MC}$ 对应的解（代价为 $N^{MC}\_{out}\bar{c}^2+r\_{in}^{MC}$）是次优的。

现在，如果我们令 $N\_{in}'=\lvert\mathcal I'\rvert$，并定义 $N'\_{out}=N'-N'\_{in}$，又因为 $\mathcal I\_{MC}$ 之外的任何一致集的大小都小于 $N^{MC}\_{in}-r^{MC}\_{in}/\bar{c}^2$ 意味着：

$$\begin{aligned}
&N_{in}'<N^{MC}_{in}-r^{MC}_{in}/\bar{c}^2\\
&\Leftrightarrow N-N'_{out}<N-N_{out}^{MC}-r^{MC}_{in}/\bar{c}^2\\
&\Leftrightarrow N'_{out}>N_{out}^{MC}+r^{MC}_{in}/\bar{c}^2
\end{aligned}$$

利用 $\eqref{12}$ 并注意到内点的残差是非负的（它是一个平方和）：

$$f_{TLS}(\mathcal I')>N’_{out}\bar{c}^2>N^{MC}_{out}\bar{c}^2+r_{in}^{MC}$$

因此，式 $\eqref{11}$ 和式 $\eqref{13}$ 不能同时满足。证毕。

