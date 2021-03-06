# Path-planning-based-on-korea-highway-information

## Overview
[2020국제대학생창작자동차경진대회](http://www.kasa.kr/cev/)에서 Path Planner 및 control부분을 맡아 진행하였던 프로젝트입니다.
path Planner 및 control은 아래 그림과 같은 구조로 이루어져있습니다.

![pathPlanner사진](https://imgur.com/i7DcVHW.jpg)

주어진 다양한  상황들에서도 자율주행을 수행하기 위해 다음과 같은 기능을 구현하였습니다.
- 경로 추종
- 주차
- 정적 장애물 회피
- 긴급 제동
- 신호등 제어

## 경로추종 (이미지를 클릭해주세요)
---
경로 생성시 Global path planner를 따로 만들지 않고 [QGIS](https://www.qgis.org/en/site/)를 사용하여 경로를 직접 그려 사용하였습니다.
차량의 현재 위치와 생성된 경로중 유클리드 거리가 가장 적은 노드를 시작 노드로 지정하고 시작 노드로부터 약 5m 앞에 있는 경로 위 점을 Look Ahead point로 지정하여 pure pursuit을 통해 제어를 합니다.

[![TrafficLight](https://img.youtube.com/vi/iTbxA9TB_SQ/0.jpg)](https://youtu.be/iTbxA9TB_SQ)
## 주차 (이미지를 클릭해주세요)
---
라이다로부터 얻은 정보로부터 각 주차 공간마다 장애물(차량)이 있는지 판단하고 그 중 최적의 주차공간에 주차를 합니다.

[![TrafficLight](https://img.youtube.com/vi/3Byd47dK9k4/0.jpg)](https://youtu.be/3Byd47dK9k4)
## 정적 장애물 회피 (이미지를 클릭해주세요)
---
도로 위 장애물이 감지되면 차량의 속도를 줄이고 Local Grid map을 생성하여 장애물을 표시합니다. 차선위에 가상의 후보 경로를 무수히 그려 그 중 충돌이 나지 않는 경로를 선택하여 주행합니다.
[![TrafficLight](https://img.youtube.com/vi/2bW5VAYirro/0.jpg)](https://youtu.be/2bW5VAYirro)
## 긴급 제동 (이미지를 클릭해주세요)
---
긴급 제동 구간에서 도로 위 장애물이 감지되면 차량이 정차를 합니다.
[![TrafficLight](https://img.youtube.com/vi/9zrem4zDIdA/0.jpg)](https://youtu.be/9zrem4zDIdA)
## 신호등 제어 (이미지를 클릭해주세요)
---
신호등 구간에 들어서면 차량의 속도를 줄이고 주황불 또는 빨간불 정보를 받으면 정차합니다. 또는 좌회전/직진 신호를 보고 차량의 정차여부를 판단합니다.

[![TrafficLight](https://img.youtube.com/vi/nC-IOIOCoeY/0.jpg)](https://youtu.be/nC-IOIOCoeY)

