Giving a sensor i and its innovation ν<sub>i</sub>(k) ∈ R<sup>n</sup> associated, the following relation is formed

**q<sub>i</sub> = ν<sup>T</sup><sub>i</sub>(k) S<sub>i</sub><sup>-1</sup>(k) ν<sub>i</sub>(k)**
with  **S<sub>i</sub>(k) = H<sub>i</sub>P(k|k − 1)H<sup>T</sup><sub>i</sub> + R<sub>i</sub>(k)**.

Where **q<sub>i</sub>** ∈ R<sup>+</sup>  is a Chi (X<sup>2</sup>) distribution with n degrees of freedom.
Plot of the Chi distribution and its cumulative distribution for different DOF is given in the figures below:

![distribution.svg](../images/Chi-square_pdf.png)
![cumulative distribution.svg](../images/Chi-square_cdf.png)
For example, considering innovations **ν<sub>i</sub>(k) ∈ R<sup>3</sup>**  with 3 DOFs and probability less than the critical value 95% and an innovation then the Mahalanobis distance should be **t<sub>i</sub> = 7.815, above which all samples should be ignored**. So, depending on the confidence level and the dimension of the measurement we can choose the Mahalanobis distance **t<sub>i</sub>**.

As we allow the user to ignore parts of the measurement, we require a dynamic adaption of the Mahalanobis distance which makes the matter more complicated.

But in general if **no measurements are ignored**, we should have:


|        | Odom(12DOF) | Imu(3DOF)| GPS(3DOF) |
|----      |----    |----   |----   |
| **90%**  | 18.549 | 6.251 | 6.251 |
| **95%**  | 21.026 | 7.815 | 7.815 |
| **97.5%**| 23.337 | 9.348 | 9.348 |

Distances for different nr of DOF can be found in [here](https://www.itl.nist.gov/div898/handbook/eda/section3/eda3674.htm).