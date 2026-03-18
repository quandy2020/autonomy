# 数学公式和代码块示例

本文档演示了数学公式和代码块复制功能的使用。

## 数学公式示例

### 行内公式

使用单个美元符号可以创建行内公式，例如：$E = mc^2$ 或 $f(x) = x^2 + 2x + 1$。

### 块级公式

使用双美元符号可以创建块级公式：

$$
\int_{-\infty}^{\infty} e^{-x^2} dx = \sqrt{\pi}
$$

或者使用 LaTeX 环境：

$ \sum_{i=1}^{n} i = \frac{n(n+1)}{2} $

### 复杂公式示例

$$
\begin{aligned}
\nabla \times \vec{\mathbf{B}} -\, \frac1c\, \frac{\partial\vec{\mathbf{E}}}{\partial t} &= \frac{4\pi}{c}\vec{\mathbf{j}} \\
\nabla \cdot \vec{\mathbf{E}} &= 4 \pi \rho \\
\nabla \times \vec{\mathbf{E}}\, +\, \frac1c\, \frac{\partial\vec{\mathbf{B}}}{\partial t} &= \vec{\mathbf{0}} \\
\nabla \cdot \vec{\mathbf{B}} &= 0
\end{aligned}
$$

## 代码块复制功能示例

### Python 代码块

```python
def hello_world():
    """打印 Hello World"""
    print("Hello, World!")
    return True

# 调用函数
result = hello_world()
```

### C++ 代码块

```cpp
#include <iostream>

int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
```

### Bash 代码块（带提示符）

```bash
$ cd /path/to/directory
$ ls -la
$ python3 script.py
```

### 带 Python 提示符的代码

```python
>>> import numpy as np
>>> arr = np.array([1, 2, 3, 4, 5])
>>> print(arr.mean())
3.0
```

## 使用说明

- **数学公式**：使用 `$...$` 创建行内公式，使用 `$$...$$` 创建块级公式
- **代码复制**：点击代码块右上角的复制按钮即可一键复制代码内容
- **提示符处理**：复制按钮会自动移除代码中的提示符（如 `>>>`, `$`, `In [1]:` 等）
