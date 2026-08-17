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
`Default lane mode` → `traffic light`→`parking` → `Stop tunnel`
<img width="720" height="464" alt="image" src="https://github.com/user-attachments/assets/4e189172-5c6c-4343-bded-6bf15d308db7" />


<br>


## 사용 기술 및 장비

* **언어 및 환경**: Python 3.10, Ubuntu 22.04, ROS2 Humble
* **도구**: Colab, VSCode
* **로봇 하드웨어**: X
* **로봇 소프트웨어**: Gazebo
