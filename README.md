# Manufacturing_of_Robotics_Physical_AI
로봇 제조 실습 프로젝트 1

### Contributors
|나승원|김연철|김익균|박효빈|
|----|----|----|----|
|**[lala4768](https://github.com/lala4768)**|**[duscjf6923-afk](https://github.com/duscjf6923-afk)**|**[iq0210](https://github.com/iq0210)**|**[parlankton](https://github.com/parlankton)**|


<br>

## 프로젝트 개요

* **주제**
  
  * Issacsim을 활용해 시뮬레이션 환경에서 Go2 사족보행로봇 주행
  * 실물 Go2 로봇을 활용해 시뮬레이션 주행 코드를 real world 에 적용시켜 SimtoReal 문제 해결
* **개발 배경**

  * 최근 로봇 산업은 시뮬레이션 환경(Issacsim, Mujoco, Gazebo 등)에서 로봇을 학습시켜 실제 로봇에 적용시킴
  * 시뮬레이션 환경에서 학습한 로봇을 실제 환경으로 불러오는 과정에서 SimtoReal Gap이 발생하기에 이를 해결하는게 주요 과제임
* **목표**

  * Issacsim 환경에서 Go2 사족보행로봇을 업로드하고 주어진 코스를 완주
  * 가상 환경에서 구현한 코드를 실제 로봇에 적용시키고 그 과정에서 발생하는 SimtoReal Gap 문제를 해결해 주어진 코스를 완주

    
<br>


# Flow Chart
* **Simulaltion environment(Issacsim)**
  * `Robot & World bringup` → `Driving the course(with obstacle avoidance)`

* **Real World**
  * `Robot bringup` → `Driving the course(with obstacle avoidance)`


<br>


## 사용 기술 및 장비

* **언어 및 환경**: Python 3.10, Ubuntu 22.04, ROS2 Humble
* **도구**: VSCode
* **로봇 하드웨어**: Go2 Quadruped robot
* **로봇 소프트웨어**: Issacsim, Docker


<br>


## 주요 기능

**Lidar를 활용한 장애물 회피 로직**
* Go2에 내장된 Lidar를 활용해 로봇 정면 기준으로 일정 거리, 일정 각도 내에 라이다에 탐지되면 반대로 회전하면서 주행하도록 로직 설계
* 로봇 회전시 라이다가 탐지한 장애물 거리가 매우 가까울 경우 정지하는 로직 추가
* 좌우로 동일한 각도에 라이다가 장애물을 인지하면 일정 거리 후진하고 다시 주행하는 로직 추가
* 회피 주행시 일반 주행보다 직선 주행 속도를 줄여 회피를 보다 자연스럽게 하도록 로직 설계




<br>


## 프로젝트 성과

* Issacsim에 robot과 world를 업로드하고 주행 코드를 적용해 로봇이 시뮬레이션상에서 정해진 코스를 장애물과 충돌없이 완주함
* Simulation 코드를 실제 로봇에 적용시키면서 발생하는 SimtoReal 문제 직면 후 주행 알고리즘 수정, 파라미터 조정 등의 방법을 활용해 이를 해결
* 실제 로봇으로 주행했을 때 course 은1 2위, course 2은 1위로 완주

* **Real World Course 1**

<img width="654" height="368" alt="map1 real world gif" src="https://github.com/user-attachments/assets/a23dd6cf-b8c3-44a5-8e4c-964b1a3b3e96" />


* **Real World Course 2**

<img width="654" height="368" alt="map2 real world gif" src="https://github.com/user-attachments/assets/ea94a8b1-e694-470a-80ce-cc8431d86bd9" />
