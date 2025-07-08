English| [简体中文](./NodeHub项目创建指南.md)

# NodeHub Project Creation Guide

## Introduction to NodeHub
NodeHub is an intelligent robot application center designed for robot enthusiasts. It aims to assist robot enthusiasts in developing their own intelligent robots in a simpler, more efficient, and open manner.

## How to Access NodeHub?
1. Open your browser and visit the following URL: https://developer.d-robotics.cc/nodehub  
2. In the DiGua Developer Community, click the NodeHub tab at the top to navigate to NodeHub.
![图片](images_2.0/1.jpg)
3. Scroll down the NodeHub page to view the related robot nodes. Click on a tag to enter the detail page of that node.
![图片](images_2.0/2.jpg)

## Create and Publish a Node
1. Enter the DiGua Developer Community NodeHub interface, scroll down, and click the "Create Project" button.
    ![图片](images_2.0/3.jpg)

2. Fill in the relevant information, and once completed, click the "Next" button. Below are the instructions for each field:
    ![图片](images_2.0/4.jpg)(1) **Project Name**: Choose a name for your Node. The name must not be the same as any existing project on NodeHub. If you enter a duplicate name, the system will notify you when you click "Next." Sometimes, even if you believe the name is unique, the system might still flag it as a duplicate. This could be due to the project having been created more than once. You can choose to stop the current creation or delete the previously created project.

  (2) **Project Description**: Provide a brief description of the Node. This content will be displayed when someone hovers over your project icon after it has been successfully published.![图片](images_2.0/5.jpg)
  (3) **Project Category**: Choose a category for your Node. This category will be used to classify the project on the NodeHub page. Note that you can select multiple categories if applicable.
  ![图片](images_2.0/6.jpg)
  ![图片](images_2.0/7.jpg)

  （4）**Code Repository**: Enter the URL of your code repository. The repository must contain the following files:

- **README_cn.md** (in Chinese): The DiGua Developer Community NodeHub will fetch this file as the content for the NodeHub page in Chinese.

- **README.md** (in English): The DiGua Developer Community NodeHub will fetch this file as the content for the NodeHub page in English.
  ![图片](images_2.0/8.jpg)

  （5）**Operating Platform**: Choose a category for the operating platform of your Node. This category will be used to classify the project on the NodeHub page. Note that you can select multiple categories if applicable.
  ![图片](images_2.0/9.jpg)
  ![图片](images_2.0/10.jpg)

  （6）**Cover Image**: Step 1: Click the "Upload Banner" button under "Cover Image." Step 2: In the pop-up file explorer, select the path to your image. Step 3: Choose your image. Step 4: Click the "Open" button. After clicking, the selection window will close. Wait a moment, and once the image is successfully uploaded, it will appear in the "Cover Image" section. This image will be displayed as the project showcase image after the project is successfully published.![图片](images_2.0/11.jpg)
  ![图片](images_2.0/12.jpg)

  （7）**Video Link**: Currently, NodeHub supports Bilibili video links. After publishing your video on Bilibili, you can obtain the link by clicking the "Copy Link" button below the video. Then, click the "Embed Code" button to copy the embed link to your clipboard. Finally, paste this link into the "Video Link" input field on NodeHub.
  ![图片](images_2.0/13.jpg)

3. Wait for NodeHub to fetch the README files from the GitHub repository. Once the fetch is complete, the message "Fetch Successful" will be displayed.
![图片](images_2.0/14.jpg)
![图片](images_2.0/15.jpg)
4. Click "Accept Apache 2.0 License Agreement," then click the "Publish" button.
![图片](images_2.0/16.jpg)
5. Once published successfully, you can find your Node on the NodeHub homepage.
![图片](images_2.0/17.jpg)



## Common Issue Troubleshooting
1. **Error after clicking "Next": "Failed to fetch NodeHub project GitLab repository files."**
You need to check whether the code repository URL is correct, and then click the "Next" button again.![图片](images_2.0/20.jpg)

2. **After clicking the "Publish" button, if you see the message "Project addition failed,"** 

  It might be due to network fluctuations or other reasons. In most cases, the project has been successfully uploaded. You can go back to the NodeHub homepage or your personal center to find the Node you just published.
  ![图片](images_2.0/21.jpg)
