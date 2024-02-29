---
title: 扫描毫米波雷达点云配准
date: '2024-01-30 17:04:53 +0800'
categories: [Research]
tags: [pointcloud registraion, non-convex optimization]
math: true
---

# 扫描毫米波雷达点云配准

> 本文讨论**适用于毫米波雷达数据的点云配准问题**。

## 扫描毫米波雷达测量模型

### 扫描毫米波雷达硬件参数特征

本文采用 Mulran 数据集，该数据集的主要目的是促进基于距离传感器的位置识别研究。为了实现这个目标，我们构建了一个传感器系统，所采用扫描毫米波雷达各项参数如下：

| Manufacturer |  Model   |             Description              |  Hz  | Range |
| :----------: | :------: | :----------------------------------: | :--: | :---: |
|   Navtech    | CIR204-H | 0.9° and 0.06 m resolution, 360° FOV |  4   | 200 m |

取数据集中扫描毫米波雷达原始数据如下：

![image-20240131111147549](assets/img/20240130/image-20240131111147549.png)

_Mulran 数据集所提供的单帧扫描毫米波雷达数据（极坐标表示 $3360\times400$）_

应用广泛的 360° FMCW Radar 传感器输出的图像，沿高度和宽度方向的轴分别表示相对于传感器框架的**径向**和**切向**。

> 注意到径向方向的分辨率远远优于切向。
> {: .prompt-tip }

### 扫描毫米波雷达各向异性不确定性建模

与激光雷达测量的各向同性（即它们沿轴的方差可能相等）不确定性不同，扫描毫米波雷达测量具有各向异性特征，也就是说，沿方位角方向的每个点的不确定性相对较大，而沿径向方向的不确定性较小。为了建模雷达特征点的不确定性，测量噪声被表示为<u>沿着扫描毫米波雷达信号发射的方向的径向不确定性</u>和<u>垂直于径向方向的方位不确定性的叠加</u>。因此，不确定性呈现为香蕉形状（如下图所示）。

![uncertainty_modeling_1](/assets/img/20240130/uncertainty_modeling_1.png){: width="972" height="589" }

_扫描毫米波雷达雷达特征点各向异性特征的可视化描述_

于是，在扫描毫米波雷达测量模型中，给定点云 $\boldsymbol P=\lbrace\boldsymbol p_k\rbrace\_{k=1}^N$，其中 $\boldsymbol p_k\in R^2$，将点云中第 $k$ 个点以极坐标形式表示，即

$$\begin{equation}\boldsymbol p_k=\rho_k\begin{bmatrix}\cos\theta_k&\sin\theta_k\end{bmatrix}^T\tag{1}\end{equation}$$

并且将测量量写为“真值+噪声项”（如 $\boldsymbol p_k=\boldsymbol p_k^\circ+\delta \boldsymbol p_k$）的形式，其优势如下：

1. **噪声项总是在原点附近的白噪声**，使后续推导中线性化近似导致的误差不至于过大。
2. **噪声项为小量**，其二阶变量相对来说可以忽略。

其中，

$$\begin{equation}\rho_k=\rho_k^\circ+\delta\rho_k\tag{2}\end{equation}$$

$\delta\rho_k$ 表示径向测量的噪声项，服从 $\delta\rho_k\sim N(0,\sigma_{\rho_k}^2)$，$\sigma_{\rho_k}$ 表示其标准差；

$$\begin{equation}\begin{bmatrix}\cos\theta_k&\sin\theta_k\end{bmatrix}^T=\begin{bmatrix}\cos(\theta_k^\circ+\delta\theta_k)&\sin(\theta_k^\circ+\delta\theta_k)\end{bmatrix}^T\tag{3}\end{equation}$$

$\delta\theta_k$ 表示切向测量的噪声项，$\delta\theta_k\sim N(0,\sigma_{\theta_k}^2)$，$\sigma_{\theta_k}$ 表示其标准差，进一步讨论，

$$\begin{equation}\begin{aligned}\begin{bmatrix}\cos\theta_k&\sin\theta_k\end{bmatrix}^T&=\begin{bmatrix}\cos(\theta_k^\circ+\delta\theta_k)&\sin(\theta_k^\circ+\delta\theta_k)\end{bmatrix}^T\\
&=\begin{bmatrix}\cos\delta\theta_k&-\sin\delta\theta_k\\\sin\delta\theta_k&\cos\delta\theta_k\end{bmatrix}\begin{bmatrix}\cos\theta_k^\circ\\\sin\theta_k^\circ\end{bmatrix}\\
&=\begin{bmatrix}\cos\delta\theta_k&-\sin\delta\theta_k\\\sin\delta\theta_k&\cos\delta\theta_k\end{bmatrix}\begin{bmatrix}\cos\theta_k^\circ\\\sin\theta_k^\circ\end{bmatrix}\\
&\approx\bigg(\begin{bmatrix}1&0\\0&1\end{bmatrix}+\begin{bmatrix}0&-\delta\theta_k\\\delta\theta_k&0\end{bmatrix}\bigg)\begin{bmatrix}\cos\theta_k^\circ\\\sin\theta_k^\circ\end{bmatrix}\end{aligned}\tag{4}\end{equation}$$

因此，将 $(2)$ 和 $(4)$ 代入 $(1)$，则点云中第 $k$ 个点的不确定度可以表示为

$$\begin{equation}\begin{aligned}\boldsymbol p_k&=(\rho_k^\circ+\delta\rho_k)\bigg(\begin{bmatrix}1&0\\0&1\end{bmatrix}+\begin{bmatrix}0&-\delta\theta_k\\\delta\theta_k&0\end{bmatrix}\bigg)\begin{bmatrix}\cos\theta_k^\circ\\\sin\theta_k^\circ\end{bmatrix}\\
&=\boldsymbol p_k^\circ+\rho_k\begin{bmatrix}0&-1\\1&0\end{bmatrix}\begin{bmatrix}\cos\theta_k^\circ\\\sin\theta_k^\circ\end{bmatrix}\delta\theta_k+\begin{bmatrix}\cos\theta_k\\\sin\theta_k\end{bmatrix}\delta\rho_k\\
&\approx \boldsymbol p_k^\circ+\rho_k\begin{bmatrix}0&-1\\1&0\end{bmatrix}\begin{bmatrix}\cos\theta_k\\\sin\theta_k\end{bmatrix}\delta\theta_k+\begin{bmatrix}\cos\theta_k\\\sin\theta_k\end{bmatrix}\delta\rho_k\end{aligned}\tag{5}\end{equation}$$

即

$$\begin{equation}\delta \boldsymbol p_k=\rho_k\begin{bmatrix}0&-1\\1&0\end{bmatrix}\begin{bmatrix}\cos\theta_k\\\sin\theta_k\end{bmatrix}\delta\theta_k+\begin{bmatrix}\cos\theta_k\\\sin\theta_k\end{bmatrix}\delta\rho_k\tag{6}\end{equation}$$

$\delta\boldsymbol p_k$ 表示点云中第 $k$ 个点坐标的噪声项，令 $C_{\boldsymbol p_k}$ 表示其协方差，则有

$$\begin{equation}C_{\boldsymbol p_k}=\rho_k^2\begin{bmatrix}\sin^2\theta_k&-\sin\theta_k\cos\theta_k\\-\cos\theta_k\sin\theta_k&\cos^2\theta_k\end{bmatrix}\sigma_{\theta}^2+\begin{bmatrix}\cos^2\theta_k&\cos\theta_k\sin\theta_k\\\cos\theta_k\sin\theta_k&\sin^2\theta_k\end{bmatrix}\sigma_{\rho}^2\tag{7}\end{equation}$$

> 由径向和切向的分辨率，可估计 $\sigma_{\theta}^2=10.8°,\sigma_{\rho}^2=0.1m$
{: .prompt-info }

也就是说，考虑上述模型，扫描毫米波雷达测量 $\boldsymbol p_k$ 近似服从 $\boldsymbol p_k\sim N(\boldsymbol p_k,C_{\boldsymbol p_k})$ 的正态分布。



## 缩放估计

我们可以简单地执行前文所描述的缩放、旋转和平移估计的级联。但对于缩放估计，由于 TIMs 能够与定义在三维点上的图的拓扑结构联系起来，可通过图论的解释进行离群值的剔除。

![image-20240201114615553](assets/img/20240130/image-20240201114615553.png)

将 TRIMs 视为完全图 $G(V, E)$ 中的边（其中顶点 $V$ 是对应关系，边集 $E$ 诱导出 TIMs 和 TRIMs）。在估计了缩放之后（<u>对于扫描毫米波雷达测量来说，连续的两帧之间缩放为 1</u>），我们可以剔除图中的边 $(i,j)$，若与其关联的 TRIM $s\_{ij}$ 被 $\bar c$ 分类为离群值（即，$|s\_{ij} −\hat s|\geq \bar c$）。这使得我们获得一个修剪过的图 $G'(V, E')$，其中 $E' ⊆ E$​，剔除了显著的离群值。

![image-20240201114629684](assets/img/20240130/image-20240201114629684.png)

可以被证明：内点 TIM 对应的边在 $E'$ 中形成一个团，且 $E'$ 中至少存在一个包含所有内点的极大团。因此，我们只需要从完全图 $G(V, E)$ 中搜索最大团（如图蓝色阴影部分），即可实现离群值的有效且快速地剔除。



## 旋转估计

对于旋转估计，我们可以通过计算 $\boldsymbol p_{ij}=\boldsymbol p_j-\boldsymbol p_i$ 和 $\boldsymbol q_{ij}=\boldsymbol q_j-\boldsymbol q_i$ 来获得平移不变度量（TIM），而 TIM 近似服从 $\boldsymbol p_{ij}\sim N(\boldsymbol p_j-\boldsymbol p_i,C_{\boldsymbol p_j}+C_{\boldsymbol p_i})$​ 的正态分布。

![uncertainty_modeling_2](/assets/img/20240130/uncertainty_modeling_2.png){: width="972" height="589" }

由《缩放、旋转和平移解耦的点云配准》可知，两帧之间平移不变度量（TIM）的夹角即为两帧之间的相对旋转，对该夹角 $\theta_{ij}=\arctan\bigg(\frac{^{(2)}q_{ij}}{^{(1)}q_{ij}}\bigg)-\arctan\bigg(\frac{^{(2)}p_{ij}}{^{(1)}p_{ij}}\bigg)$ 做截断最小二乘估计并应用自适应投票方法，其中 $^{(l)}(\cdot)$ 表示该向量的第 $l$ 个元素，$l=1,2$​。然而，要使用自适应投票法从若干平移不变度量（TIM）的测量量中估计真值，还需已知测量量的不确定性。两帧之间平移不变度量（TIM）的夹角 $\theta$ 的表达式是非线性的， TIM 近似服从的正态分布经过该式后讲难以被解析地描述。为此，可以考虑近似，将平移不变量度量的协方差矩阵投影至其切向（角度方向），以此作为角度不确定性的定性描述。

首先，需要将 $\boldsymbol p_{ij}$ 和 $\boldsymbol q_{ij}$ 的协方差矩阵分别旋转至以 $\boldsymbol p_{ij}$ 和 $\boldsymbol q_{ij}$ 方向为第二坐标轴方向的坐标系下，设相应旋转矩阵为 $\boldsymbol R_{\boldsymbol p_{ij}}$ 和 $\boldsymbol R_{\boldsymbol q_{ij}}$，

$$\begin{equation}\boldsymbol R_{\boldsymbol p_{ij}}=\begin{bmatrix}\cos\bigg(\frac{\pi}{2}-\arctan\bigg(\frac{^{(2)}p_{ij}}{^{(1)}p_{ij}}\bigg)\bigg)&-\sin\bigg(\frac{\pi}{2}-\arctan\bigg(\frac{^{(2)}p_{ij}}{^{(1)}p_{ij}}\bigg)\bigg)\\\sin\bigg(\frac{\pi}{2}-\arctan\bigg(\frac{^{(2)}p_{ij}}{^{(1)}p_{ij}}\bigg)\bigg)&\cos\bigg(\frac{\pi}{2}-\arctan\bigg(\frac{^{(2)}p_{ij}}{^{(1)}p_{ij}}\bigg)\bigg)\end{bmatrix}\tag{8}\end{equation}$$

$$\begin{equation}C_{\boldsymbol p_{ij}}'=\boldsymbol R_{\boldsymbol p_{ij}}^TC_{\boldsymbol p_j}\boldsymbol R_{\boldsymbol p_{ij}}\tag{9}\end{equation}$$

对于 $\boldsymbol R_{\boldsymbol q_{ij}}$ 同理，从略。

那么协方差矩阵在此坐标系下对第一轴的边缘分布（取 $C_{\boldsymbol p_{ij}}'$ $(1,1)$ 处元素为方差 $\sigma_{\boldsymbol p_{ij}}^2$）即原分布在切向方向的投影。不过，仍然要注意到，此方差为对 $\boldsymbol p_{ij}$ 的不确定性在切向方向的笛卡尔描述，还需要将其转为角度描述，

简单起见，将标准差作为对边，$\boldsymbol p_{ij}$ 作为邻边，求其夹角即可，

$$\begin{equation}^\theta\sigma_{\boldsymbol p_{ij}}=\arctan\bigg(\frac{\sigma_{\boldsymbol p_{ij}}}{\|\boldsymbol p_{ij}\|}\bigg)\tag{10}\end{equation}$$

两帧之间平移不变度量（TIM）的夹角 $\theta_{ij}$ 是测量值，同样可以表达为“真值+噪声项”（如 $\theta_{ij}=\theta_{ij}^\circ+\delta \theta_{ij}$）的形式，其中

$$\begin{equation}\theta_{ij}^\circ=\arctan\bigg(\frac{^{(2)}q_{ij}^\circ}{^{(1)}q_{ij}^\circ}\bigg)-\arctan\bigg(\frac{^{(2)}p_{ij}^\circ}{^{(1)}p_{ij}^\circ}\bigg)\tag{11}\end{equation}$$

$$\begin{equation}\delta\theta_{ij}\sim N(0{,}^\theta\sigma_{\boldsymbol p_{ij}}^2{+}^\theta\sigma_{\boldsymbol q_{ij}}^2)\tag{12}\end{equation}$$

得到如上带不确定性的相对旋转测量量，即可使用自适应投票法对旋转进行鲁棒估计。



## 平移估计

对于平移估计，在求出旋转角度 $\hat\theta$ 后，可以将其写为旋转矩阵 $\hat{\boldsymbol R}$，

$$\begin{equation}\hat{\boldsymbol R}=\begin{bmatrix}\cos(\hat\theta)&-\sin(\hat\theta)\\\sin(\hat\theta)&\cos(\hat\theta)\end{bmatrix}\tag{13}\end{equation}$$

由《缩放、旋转和平移解耦的点云配准》可知，两帧之间的相对平移可以表示为 $\boldsymbol t_i=\boldsymbol q_i-\hat{\boldsymbol R}\boldsymbol p_i$。同样，希望使用自适应投票法解决该截断最小二乘估计问题，由于表达式为线性，易得相对平移测量值的不确定性为 $C_{\boldsymbol q_i}+\hat{\boldsymbol R}C_{\boldsymbol p_i}\hat{\boldsymbol R}^T$。

对于相对平移可以表示为向量形式，其两个维度的估计是不相关的，因此，可以将一个二维的向量估计简化为两个标量分别估计。对于第一维度，其测量量的方差为 $C_{\boldsymbol q_i}+\hat{\boldsymbol R}C_{\boldsymbol p_i}\hat{\boldsymbol R}^T$ 矩阵的 $(1,1)$ 处元素 $^{(1)}\sigma_i^2$；对于第二维度，其测量量的方差为 $(2,2)$ 处元素 $^{(2)}\sigma_i^2$。将相对平移 $\boldsymbol t_i$ 表示为 $^{(1)}t_i$ 和 $^{(2)}t_i$，同样可以表达为“真值+噪声项”的形式，其中

$$\begin{equation}^{(l)}t_i^\circ{=}^{(l)}\{\boldsymbol q_i^\circ-\hat{\boldsymbol R}\boldsymbol p_i^\circ\},\ l=1,2,\tag{14}\end{equation}$$

$$\begin{equation}\delta^{(l)}t_i\sim N(0{,}^{(l)}\sigma_i^2),\ l=1,2,\tag{15}\end{equation}$$

得到如上带不确定性的相对平移测量量，即可使用自适应投票法对平移进行鲁棒估计。

