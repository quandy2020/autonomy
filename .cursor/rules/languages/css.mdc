---
description: CSS 和样式规范
globs: *.css, *.scss, *.less, *.styled.ts
alwaysApply: false
---

# CSS 和样式规范

## 样式架构原则
- **组件化样式**：每个组件的样式应该封装在组件内部
- **样式隔离**：避免全局样式污染，使用CSS-in-JS或CSS Modules
- **主题一致性**：使用设计系统和主题变量保持视觉一致性
- **响应式设计**：优先考虑移动端，采用移动优先的响应式设计
- **性能优化**：避免不必要的样式重绘和重排

## Styled Components 规范
- **组件命名**：使用描述性的组件名，以 `Styled` 开头
  ```typescript
  const StyledCard = styled.div`
    padding: 16px;
    border-radius: 8px;
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
  `;
  ```

- **主题使用**：通过 `theme` 属性访问主题变量
  ```typescript
  const StyledButton = styled.button`
    background-color: ${({ theme }) => theme.colors.primary};
    color: ${({ theme }) => theme.colors.white};
  `;
  ```

- **条件样式**：使用 props 进行条件样式设置
  ```typescript
  const StyledButton = styled.button<{ variant: 'primary' | 'secondary' }>`
    background-color: ${({ variant, theme }) =>
      variant === 'primary' ? theme.colors.primary : theme.colors.secondary
    };
  `;
  ```

- **样式继承**：合理使用样式继承减少重复代码
  ```typescript
  const BaseButton = styled.button`
    padding: 8px 16px;
    border-radius: 4px;
    border: none;
  `;

  const PrimaryButton = styled(BaseButton)`
    background-color: ${({ theme }) => theme.colors.primary};
  `;
  ```

## Ant Design 定制规范
- **主题定制**：使用 ConfigProvider 进行全局主题定制
  ```typescript
  const theme = {
    token: {
      colorPrimary: '#1890ff',
      borderRadius: 6,
      fontSize: 14,
    },
  };
  ```

- **组件样式覆盖**：使用 CSS-in-JS 覆盖 Ant Design 组件样式
  ```typescript
  const StyledTable = styled(Table)`
    .ant-table-thead > tr > th {
      background-color: #fafafa;
      font-weight: 600;
    }
  `;
  ```

- **自定义组件**：基于 Ant Design 组件创建自定义组件
  ```typescript
  const CustomCard = styled(Card)`
    border-radius: 12px;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
  `;
  ```

## 响应式设计规范
- **断点定义**：使用标准断点进行响应式设计
  ```typescript
  const breakpoints = {
    xs: '480px',
    sm: '576px',
    md: '768px',
    lg: '992px',
    xl: '1200px',
    xxl: '1600px',
  };
  ```

- **媒体查询**：使用 CSS-in-JS 编写媒体查询
  ```typescript
  const ResponsiveContainer = styled.div`
    padding: 16px;

    @media (min-width: ${({ theme }) => theme.breakpoints.md}) {
      padding: 24px;
    }
  `;
  ```

- **Flex布局**：优先使用 Flexbox 进行布局
  ```typescript
  const FlexContainer = styled.div`
    display: flex;
    flex-direction: column;
    gap: 16px;

    @media (min-width: ${({ theme }) => theme.breakpoints.md}) {
      flex-direction: row;
    }
  `;
  ```

## 颜色和主题规范
- **颜色系统**：定义完整的颜色系统
  ```typescript
  const colors = {
    primary: '#1890ff',
    success: '#52c41a',
    warning: '#faad14',
    error: '#ff4d4f',
    text: {
      primary: '#262626',
      secondary: '#595959',
      disabled: '#bfbfbf',
    },
    background: {
      primary: '#ffffff',
      secondary: '#fafafa',
      disabled: '#f5f5f5',
    },
  };
  ```

- **暗色主题**：支持暗色主题切换
  ```typescript
  const darkTheme = {
    colors: {
      primary: '#1890ff',
      background: {
        primary: '#141414',
        secondary: '#1f1f1f',
      },
      text: {
        primary: '#ffffff',
        secondary: '#a6a6a6',
      },
    },
  };
  ```

## 动画和过渡规范
- **过渡效果**：为交互元素添加适当的过渡效果
  ```typescript
  const AnimatedButton = styled.button`
    transition: all 0.3s ease;

    &:hover {
      transform: translateY(-2px);
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
    }
  `;
  ```

- **加载动画**：使用 CSS 动画创建加载效果
  ```typescript
  const LoadingSpinner = styled.div`
    @keyframes spin {
      0% { transform: rotate(0deg); }
      100% { transform: rotate(360deg); }
    }

    animation: spin 1s linear infinite;
  `;
  ```

## 布局规范
- **网格系统**：使用 CSS Grid 或 Flexbox 创建网格布局
  ```typescript
  const GridContainer = styled.div`
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 24px;
  `;
  ```

- **间距系统**：使用统一的间距系统
  ```typescript
  const spacing = {
    xs: '4px',
    sm: '8px',
    md: '16px',
    lg: '24px',
    xl: '32px',
    xxl: '48px',
  };
  ```

## 字体和排版规范
- **字体系统**：定义完整的字体系统
  ```typescript
  const typography = {
    fontFamily: {
      primary: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto',
      mono: '"SFMono-Regular", Consolas, "Liberation Mono", Menlo',
    },
    fontSize: {
      xs: '12px',
      sm: '14px',
      md: '16px',
      lg: '18px',
      xl: '20px',
      xxl: '24px',
    },
    fontWeight: {
      normal: 400,
      medium: 500,
      semibold: 600,
      bold: 700,
    },
  };
  ```

- **行高和字间距**：设置合适的行高和字间距
  ```typescript
  const TextComponent = styled.p`
    line-height: 1.6;
    letter-spacing: 0.02em;
  `;
  ```

## 性能优化规范
- **CSS优化**：避免深层嵌套和复杂选择器
- **重绘重排**：避免频繁的样式变更导致的重绘重排
- **CSS-in-JS优化**：使用 `shouldForwardProp` 避免不必要的 DOM 属性
  ```typescript
  const StyledDiv = styled.div.withConfig({
    shouldForwardProp: (prop) => !['customProp'].includes(prop),
  })<{ customProp: boolean }>`
    color: ${({ customProp }) => customProp ? 'red' : 'blue'};
  `;
  ```

## 可访问性规范
- **对比度**：确保文本和背景有足够的对比度
- **焦点状态**：为可交互元素提供清晰的焦点状态
  ```typescript
  const AccessibleButton = styled.button`
    &:focus {
      outline: 2px solid ${({ theme }) => theme.colors.primary};
      outline-offset: 2px;
    }
  `;
  ```

- **语义化**：使用语义化的 HTML 元素和 ARIA 属性

## 代码组织规范
- **文件结构**：样式文件与组件文件放在同一目录
- **样式分离**：将复杂的样式逻辑提取到单独的样式文件
- **主题文件**：将主题相关的配置集中管理
- **工具函数**：创建样式工具函数提高复用性
  ```typescript
  const getSpacing = (size: keyof typeof spacing) => spacing[size];
  const getColor = (color: string) => ({ theme }: { theme: any }) => theme.colors[color];
  ```