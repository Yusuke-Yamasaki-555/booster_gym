はい、承知いたしました。提供されたPythonコードとYAMLファイルから、強化学習で用いられる報酬関数を数式で示し、対応するLaTeXコードを提示します。

このプログラムにおける総報酬 $R$ は、各報酬項 $r_i$ にそれぞれの重み係数 $w_i$ を乗算したものの総和として計算されます。

$$R = \sum_{i \in \text{reward terms}} w_i r_i$$

YAMLファイルの設定で `only_positive_rewards` が `true` の場合、上記の総和が負であれば、最終的な報酬は 0 になります。

$$R_{\text{final}} = \max(0, R)$$

---

## 報酬関数の数式

以下に、報酬関数を構成する各項とその重み係数を示します。

$$
\begin{aligned}
R &= 0.25 \cdot r_{\text{survival}} \\
  &+ 1.0 \cdot r_{\text{track\_x}} \\
  &+ 1.0 \cdot r_{\text{track\_y}} \\
  &+ 0.5 \cdot r_{\text{track\_z}} \\
  &- 20.0 \cdot r_{\text{height}} \\
  &- 5.0 \cdot r_{\text{orient}} \\
  &- 2.0 \times 10^{-4} \cdot r_{\text{torques}} \\
  &- 1.0 \times 10^{-2} \cdot r_{\text{tired}} \\
  &- 2.0 \times 10^{-3} \cdot r_{\text{power}} \\
  &- 2.0 \cdot r_{\text{vel\_z}} \\
  &- 0.2 \cdot r_{\text{ang\_vel\_xy}} \\
  &- 1.0 \times 10^{-4} \cdot r_{\text{dof\_vel}} \\
  &- 1.0 \times 10^{-7} \cdot r_{\text{dof\_acc}} \\
  &- 1.0 \times 10^{-4} \cdot r_{\text{root\_acc}} \\
  &- 1.0 \cdot r_{\text{action\_rate}} \\
  &- 1.0 \cdot r_{\text{dof\_pos\_lim}} \\
  &- 1.0 \cdot r_{\text{collision}} \\
  &- 0.1 \cdot r_{\text{slip}} \\
  &- 1.0 \cdot r_{\text{yaw\_diff}} \\
  &- 1.0 \cdot r_{\text{yaw\_mean}} \\
  &- 0.1 \cdot r_{\text{roll}} \\
  &- 1.0 \cdot r_{\text{dist}} \\
  &+ 3.0 \cdot r_{\text{swing}}
\end{aligned}
$$

### 各報酬項の定義

-   **生存報酬 (Survival)**
    -   $r_{\text{survival}} = 1$
-   **目標速度追従 (Tracking Velocity)**
    -   $r_{\text{track\_x}} = \exp\left(-\frac{(v_x^{\text{cmd}} - \hat{v}_x)^2}{\sigma_{\text{track}}}\right)$
    -   $r_{\text{track\_y}} = \exp\left(-\frac{(v_y^{\text{cmd}} - \hat{v}_y)^2}{\sigma_{\text{track}}}\right)$
    -   $r_{\text{track\_z}} = \exp\left(-\frac{(\omega_z^{\text{cmd}} - \hat{\omega}_z)^2}{\sigma_{\text{track}}}\right)$
-   **胴体の高さ維持 (Base Height)**
    -   $r_{\text{height}} = (h_{\text{base}} - h_{\text{target}})^2$
-   **衝突ペナルティ (Collision)**
    -   $r_{\text{collision}} = \sum_{i \in I_{\text{penalized}}} \mathbb{I}(\|F_{c,i}\| > 1.0)$
-   **Z軸方向の線形速度ペナルティ (Linear Velocity Z)**
    -   $r_{\text{vel\_z}} = \hat{v}_z^2$
-   **XY軸周りの角速度ペナルティ (Angular Velocity XY)**
    -   $r_{\text{ang\_vel\_xy}} = \omega_x^2 + \omega_y^2$
-   **胴体の傾きペナルティ (Orientation)**
    -   $r_{\text{orient}} = \|g_{xy}\|^2$
-   **トルクペナルティ (Torques)**
    -   $r_{\text{torques}} = \sum_j \tau_j^2$
-   **関節角速度ペナルティ (DoF Velocity)**
    -   $r_{\text{dof\_vel}} = \sum_j \dot{q}_j^2$
-   **関節角加速度ペナルティ (DoF Acceleration)**
    -   $r_{\text{dof\_acc}} = \sum_j \left( \frac{\dot{q}_{j,t-1} - \dot{q}_{j,t}}{\Delta t} \right)^2$
-   **胴体加速度ペナルティ (Root Acceleration)**
    -   $r_{\text{root\_acc}} = \left\| \frac{v_{\text{root},t-1} - v_{\text{root},t}}{\Delta t} \right\|^2$
-   **行動変化率ペナルティ (Action Rate)**
    -   $r_{\text{action\_rate}} = \sum_j (a_{j,t-1} - a_{j,t})^2$
-   **関節可動域制限ペナルティ (DoF Position Limits)**
    -   $r_{\text{dof\_pos\_lim}} = \sum_j \mathbb{I}(q_j < q_j^{\text{soft\_lower}} \lor q_j > q_j^{\text{soft\_upper}})$
-   **足の滑りペナルティ (Feet Slip)**
    -   $r_{\text{slip}} = \sum_{k \in \{\text{feet}\}} \|v_{\text{foot},k}\|^2 \cdot c_{\text{foot},k}$
-   **両足のヨー角差ペナルティ (Feet Yaw Difference)**（間違い？）
    -   $r_{\text{yaw\_diff}} = (\text{normalize}(\psi_{\text{foot}, R} - \psi_{\text{foot}, L}))^2$
-   **胴体と足の平均ヨー角差ペナルティ (Feet Yaw Mean)**（間違い？）
    -   $r_{\text{yaw\_mean}} = (\text{normalize}(\psi_{\text{base}} - \bar{\psi}_{\text{feet}}))^2$
-   **足のロール角ペナルティ (Feet Roll)**
    -   $r_{\text{roll}} = \sum_{k \in \{\text{feet}\}} \phi_{\text{foot},k}^2$
-   **足間距離ペナルティ (Feet Distance)**
    -   $r_{\text{dist}} = \text{clip}(d_{\text{ref}} - d_{\text{feet}}, 0, 0.1)$
-   **足のスイング報酬 (Feet Swing)**（間違い？）
    -   $r_{\text{swing}} = \sum_{k \in \{\text{feet}\}} \mathbb{I}(s_k \land \neg c_k)$
-   **トルク疲労度ペナルティ (Torque Tiredness)**
    -   $r_{\text{tired}} = \sum_j \left( \min(1.0, |\tau_j / \tau_{\text{limit},j}|) \right)^2$
-   **消費電力ペナルティ (Power)**
    -   $r_{\text{power}} = \sum_j \max(0, \tau_j \dot{q}_j)$

---

## LaTeXコード

以下に、上記の数式を表現するためのLaTeXコードを記載します。

```latex
$$
\begin{aligned}
R &= 0.25 \cdot r_{\text{survival}} \\
  &+ 1.0 \cdot r_{\text{track\_x}} \\
  &+ 1.0 \cdot r_{\text{track\_y}} \\
  &+ 0.5 \cdot r_{\text{track\_z}} \\
  &- 20.0 \cdot r_{\text{height}} \\
  &- 5.0 \cdot r_{\text{orient}} \\
  &- 2.0 \times 10^{-4} \cdot r_{\text{torques}} \\
  &- 1.0 \times 10^{-2} \cdot r_{\text{tired}} \\
  &- 2.0 \times 10^{-3} \cdot r_{\text{power}} \\
  &- 2.0 \cdot r_{\text{vel\_z}} \\
  &- 0.2 \cdot r_{\text{ang\_vel\_xy}} \\
  &- 1.0 \times 10^{-4} \cdot r_{\text{dof\_vel}} \\
  &- 1.0 \times 10^{-7} \cdot r_{\text{dof\_acc}} \\
  &- 1.0 \times 10^{-4} \cdot r_{\text{root\_acc}} \\
  &- 1.0 \cdot r_{\text{action\_rate}} \\
  &- 1.0 \cdot r_{\text{dof\_pos\_lim}} \\
  &- 1.0 \cdot r_{\text{collision}} \\
  &- 0.1 \cdot r_{\text{slip}} \\
  &- 1.0 \cdot r_{\text{yaw\_diff}} \\
  &- 1.0 \cdot r_{\text{yaw\_mean}} \\
  &- 0.1 \cdot r_{\text{roll}} \\
  &- 1.0 \cdot r_{\text{dist}} \\
  &+ 3.0 \cdot r_{\text{swing}}
\end{aligned}
$$