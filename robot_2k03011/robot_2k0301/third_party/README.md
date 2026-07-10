# 第三方库目录

这里放逐飞 LS2K0301 官方库。

官方库已经克隆到：

```text
third_party/LS2K0301_Library
```

当前使用的官方库提交：

```text
f9000bd38
```

官方库内的用户态开源工程路径：

```text
third_party/LS2K0301_Library/LS2K030x_Library/Seekfree_LS2K030x_Opensource_Library
```

核心库目录：

```text
.../libraries/zf_common
.../libraries/zf_driver
.../libraries/zf_device
.../libraries/zf_components
```

后续更新官方库，在 `robot_2k0301` 目录执行：

```bash
./scripts/fetch_ls2k0301_library.sh
```
