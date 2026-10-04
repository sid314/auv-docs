+++
date = '{{ .Date }}'
draft = true
title = '{{ replace .File.ContentBaseName "-" " " | title }}'
author = 'Your Name'
tags = []
+++

Write a one or two sentence abstract here. It appears at the top of the post and on the home page.

<!--more-->

## Section

Inline math like $E = mc^2$ and display math:

$$
\int_{-\infty}^{\infty} e^{-x^2}\, dx = \sqrt{\pi}
$$

```python
print("hello")
```

## Takeaways

1. First point.
2. Second point.
