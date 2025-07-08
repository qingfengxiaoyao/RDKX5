# NodeHub项目创建指南

NodeHub是基于TogetheROS.Bot的全开源机器人应用中心,点击NodeHub首页上的“创建项目”按钮开启NodeHub项目开发之旅

![create](images/nodehub_create_node.png)

NodeHub项目创建分为三个主要步骤，分别是“创建项目”、“上传代码”、“预览发布”，其中

- 创建项目，该步骤主要是设置项目名称、仓库名称以及展示用于展示项目内容的图片和视频
- 上传代码，该步骤主要是将个人开发代码及使用说明上传到创建好的代码仓库中，后台自动对上传的代码进行编译检查和打包以确保该项目的可用性。项目详情页面的说明文档对应代码仓库中的README.md文档。
- 预览发布，该步骤用于预览项目真实对外展示页面，对于不满意的地方可以返回前两步进行修改

## 创建项目

![create](images/nodehub_step1.png)

### 项目名称

建议采用有吸引力、凝练的标题，这样会有更多的爱好者点击查看该项目

### 项目简介

简明扼要地说明该项目的内容及应用场景，便于其他爱好者快速理解该项目，总字数不得超过100字符。

### 项目分类

项目分类按照项目最终使用场景及负责度进行划分，支持多选，当前一共有四个标签分别是：

- 综合应用，功能及技术上具有较高的复杂度，通常包含2个及以上的其他项目，通过不同Node的组合搭完成复杂的任务
- 环境感知，感知类功能用于判断周围环境，帮助机器人更好的移动，比如常见的SLAM、深度感知、目标检测、BEV感知等都属于该类别
- 人机交互，用于与人进行交互的机器人技术，包括语音识别、手势识别、骨骼关键点检测等属于此范畴
- 外设适配，与具体硬件相关的项目，通常是针对该硬件或者同类型硬件提供的驱动程序，支持接入Together.Bot的标准消息

### 代码仓库

代码仓库（repo）名称，即最终github上仓库的名字。代码仓库可由英文字母（a~z,A~Z），数字（0~9），下划线（_）和连字符（-）组成，当点击下一步创建项目后，该名称不可更改

### 运行平台

目前Together.Bot支持两款在售的硬件平台分别是RDK X3和RDK Ultra，根据项目实际运行的平台勾选对应的平台信息

### 封面图片

该图片将用于在NodeHub首页、项目列表页面最为项目的展示页面，图片内容应能够直观反映项目核心内容，可采用GIF动图方式增加吸引力，如下图所示。建议分辨率高于1280*960，长宽比符合4:3，大小不超过6MB。

![frontpage](images/nodehub_frontpage.png)

### 视频链接

该视频用于在项目详情页面直观展示项目的内容，目前仅支持[bilibili](https://www.bilibili.com/)网站视频的嵌入。将视频上传至B站后，打开对应的视频链接。如下图所示依次点击“点击复制链接”和“嵌入代码”，B站会将该视频链接拷贝至剪切板。最后将剪切板中的内容粘贴至该处完成视频嵌入。

![bilibili](images/nodehub_bilibili_share.png)

## 上传代码

**注意1：开始正式上传代码之前需要具有github账号,并保持登陆状态**

**注意2：开发环境中需要已安装配置git环境**

NodeHub代码提交流程与常见的开源项目提交流程一致，如下图所示：

![frontpage](images/nodehub_pr.png)

代码提交流程主要分为四个步骤：

1. Fork NodeHub代码至个人代码仓库
2. 从个人代码仓库clone代码至本地进行开发
3. 将本地开发完的代码push至个人代码仓库
4. 提交Pull Requst

在该环节，有两个内容是必须的分别是：工程源码和README.md。其中工程源码用于编译构建整个项目，README.md文档用于向他人介绍项目的功能，使用方式等基本内容。

### Fork仓库

点击项目链接跳转至github代码仓库，可以看到repo中已添加了默认的README.md文件

![repo](images/repo_addr.png)

点击右上角“Fork”按钮

![fork](images/fork.png)

通常保持默认值，点击右下方“Creat fork”将nodehubs中的repo fork至个人代码仓库中

![fork](images/fork2.png)

可以看到repo已经出现在个人代码仓库中

![fork](images/fork3.png)

### Clone代码

在个人代码仓库中找到对应的repo依次点击“code”、“copy”，复制repo地址至剪切板

![clone](images/clone1.png)

使用git clone [repo地址]方式将代码clone到本地，并进行开发和commit。

![clone](images/clone2.png)

### 提交代码

使用git push将代码由本地仓库推送至github个人仓库

![push](images/push1.png)

### 提交Pull Request

打开github个人仓库页面，点击“Pull request”，进入Pull request页面

![pull request](images/pull_request1.png)

在Pull request页面点击“New pull request”，创建新的pull request

![pull request](images/pull_request2.png)

接下俩github展现当前pull request的具体信息，包括:
- pull request的源代码仓库和目标代码仓库，这里源代码仓库是carry-agi/example_nodehub_create，目标代码仓库是nodehubs/example_nodehub_create
- commit信息及修改的文件信息
- 贡献者信息

![pull request](images/pull_request3.png)

点击“Create pull request”进入下一步，

![pull request](images/pull_request4.png)

填入本次pull request的内容信息点击“Create pull request”完成pull request提交

![pull request](images/pull_request5.png)

上图可以看到刚提交的pull request处于Open状态，NodeHub管理与会在后台对代码及README.md进行审核，如果没有问题会merge到nodehubs的代码仓库。

## 预览发布

完成提交pull request后，就可以对项目进行预览，点击“预览刷新”展现项目最新状态

![preview](images/preview.png)

如果预览中有不满意的地方，可以通过点击“上一步”回到之前的“创建项目”或者“上传代码”步骤进行修改，如果预览内容均符合预期，勾选“接受apache 2.0 license”，然后点击发布。Apache Licence 是著名的非盈利开源组织 Apache 采用的协议。该协议和 BSD 类似，同样鼓励代码共享和尊重原作者的著作权，同样允许代码修改，再发布（作为开源或商业软件）。

![release](images/release.png)

提交后等待管理员审核，通过之后，项目就会呈现在NodeHub项目列表中。