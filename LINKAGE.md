# 航线规划与巡检知识助手联动

两个项目独立运行，采用同一浏览器标签页双向跳转。

| 系统 | 前端开发地址 | 后端 |
| --- | --- | --- |
| 巡检知识助手 | http://127.0.0.1:5173/ | 127.0.0.1:8010 |
| 航线规划 | http://127.0.0.1:5174/ | 127.0.0.1:8000 |

前端开启 strictPort，端口占用时明确报错，避免自动换端口导致入口指向错误应用。

## 启动

在当前双项目 PyCharm 窗口中，右上角选择 **一键启动全部（双项目前后端 + Neo4j）**，点击运行即可同时启动两套前后端和 Neo4j。

该 Compound 配置位于 `.run/All_Projects.run.xml`，包含四个持续运行的服务配置。知识助手后端使用原有 `run_backend.py`：先启动或复用 Neo4j，等待 HTTP 7474 / Bolt 7687 就绪，再执行数据库迁移、启动后台 worker 和 8010 后端。因此无需另起一个 Neo4j 启动项，避免并发重复启动数据库。Neo4j 作为共享后台数据库保留运行，停止前后端不会关闭它；需要关闭时使用知识助手项目的 `scripts/stop_neo4j.ps1`。

端口固定为知识助手前端 5173、后端 8010，航线规划前端 5174、后端 8000；两套 Vite 代理分别连接各自后端，前端端口占用时直接报错，不会悄悄切换地址。重复运行时按 PyCharm 提示重新运行已有服务即可，不要另外启动相同端口的副本。知识助手配置使用相对当前主项目的 `../ai_tools/llm-graph-builder` 路径，以及已附加的 `ai_tools` 模块 Python 环境。

知识助手按原方式启动：运行 `D:\ai_tools\llm-graph-builder\tools\pycharm\run_backend.py` 和 `run_frontend.py`，或原来的 Full Stack 组合配置。

航线规划分别启动后端和前端：

```powershell
cd D:\aaachange_demo_custom_weight
python main.py
```

```powershell
cd D:\aaachange_demo_custom_weight\frontend
npm run dev
```

使用原来配置好的 Python 环境运行各自后端。右下角机器人可打开知识助手；登录知识助手后，左侧“扩展技能 → 航线规划”可返回。窄屏或折叠侧栏时仅显示图标，可通过标题查看名称。

## 可选地址配置

已有 `.env` 不用替换。仅在需要改变目标地址时追加：

- 航线规划 `frontend/.env.local`：`VITE_ASSISTANT_URL=http://127.0.0.1:5173/`
- 知识助手 `frontend/.env.local`：`VITE_PLANNING_URL=http://127.0.0.1:5174/`

修改后重启 Vite；生产部署需重新构建。航线规划如果通过 FastAPI 的 8000 端口访问，知识助手的 VITE_PLANNING_URL 应改为 `http://127.0.0.1:8000/` 并重新构建。

## 行为和范围

- 小机器人具有悬浮、眨眼、挥手和悬停提示；支持键盘焦点和减少动态效果偏好。
- 跳转前用最长 3.5 秒的网页请求检查目标可达性，失败留在当前页面并提示目标地址。跨源请求为 no-cors，仅证明目标有响应，不能确认后端业务服务和登录状态。
- 离场时页面向入口收拢，光环扩散；到达时通过一次性的 #system-portal 标记播放揭幕，随后清除标记。浏览器后退恢复页面时清理离场状态。
- 两边统一使用 admin 账号，历史上传归属已迁移。每个系统第一次需要登录，之后通过各自的持久会话恢复登录；退出、过期或清除浏览器 Cookie 后需要重新登录。不在跳转 URL 中传递密码或令牌。
- 跳转会离开当前页面；航线规划未保存的临时编辑应先按原有流程保存。

联动代码位于两项目的 `frontend/src/linkage/`。SystemPortal 的 TSX 与 JSX 版本及 CSS 实现相同交互，后续调整公共行为时需要同步两份。

## 2026-11-16 账号与分支整合

两个项目继续使用独立 Git 仓库、独立后端和前端，保留现有 PyCharm 一键运行。访问时统一使用 `127.0.0.1`，不要混用 `localhost`，浏览器按主机分别保存 Cookie。

航线规划新增 HttpOnly `planning_session` Cookie、30 天有效的本地 SQLite 会话，以及 `/api/auth/me` 和 `/api/auth/logout`。跳转返回、刷新和后端重启后可以恢复登录；退出会撤销服务端会话。管理员默认进入规划工作台，可在用户菜单打开管理后台并返回。知识助手沿用独立的 `p1_access_token` / `p1_refresh_token`，默认刷新有效期调整为 30 天（可由本地 `JWT_REFRESH_DAYS` 覆盖）。两个系统都首次登录一次；这不是跳过首次登录的单点登录。

历史上传统一归属 admin：航线规划原用户文件经 SHA-256 校验复制到 admin，同名同内容文件复用；原目录归档到仓库外。知识助手保留资源 ID、文件路径和版本内容，更新 MySQL 资源归属和 Neo4j ownership，旧用户停用并撤销旧刷新令牌，审计用户行保留。账号密码仅设置到本地账号存储，不写入源码。

一次性迁移入口是 `scripts/consolidate_admin.py`：通过 `ADMIN_PASSWORD` 提供密码，`--backup` 必须指向两个仓库之外；先检查备份与唯一键/文件冲突，再用 `--apply` 执行。执行前停止两个应用后端，保持 MySQL/Neo4j 运行。迁移前备份及报告位于 `D:/project-backups/2026-11-16/accounts`，其中包含私有数据，不上传 GitHub。

两个仓库当前版本均整理到 main。后续每项修改先 `git switch -c codex/<task>`，验证后合回 main 并推送，再清理已合并分支。旧功能分支保留为 `archive/2026-11-16-*` 标签。航线规划的 `app_change` 仍被另一个有未提交工作的 worktree 使用，仅保留该本地分支，云端旧分支已删除。
