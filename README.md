# Этот репозиторий содержит симуляцию колесного робота, следующего по линии
Так как лабораторная работа выполнена в одиночестве, то ролей нет

Постановка задачи:
Следование по линии 4-колесным мобильным роботом с фронтальной камерой

Выбранный алгоритм решения:
Кратко: ПИД-регулятор на поворот. На входе - центр масс черных пикселей в нижней части изображения. Скорость повышается при движении по прямой
- принимается изображение с камеры
- к нижним 100 пикселям применяется бинарный фильтр, затем определяется центр масс черных пикселей
- горизонтальное расстояние от центра изображения до центра масс - это функция ошибки
- функция ошибки используется в ПИД-регуляторе для получения изменения направления в диапазоне от -1 до +1
- скорость поворота прямо пропорционально модулю изменения направления
- скорость движения обратно пропорциональна модулю изменения направления (быстрее на прямых и медленнее на поворотах)

Выбранное окружение:
- ROS Noetic - текущая поддерживаемая версия
- Gazebo11
- Python3
- контейнер Docker с Ubuntu20.04

**Работа сделана Михаилом Болотовым, номер ИСУ 410969**
---
# This repository contains a simulation of a line following rover
Since the lab was done alone, there are no roles

Problem statement:
Line following by a 4-wheeled rover with a front camera

Selected solution algorithm:
Brief: PID controller for turning. Input is center of mass of black pixels at the bottom of the image. Speed ​​increases when moving in a straight line
- receives image from camera
- applies binary filter to bottom 100 pixels, then determines center of mass of black pixels
- horizontal distance from center of image to center of mass - this is error function
- error function is used in PID controller to obtain direction change in range from -1 to +1
- rotation speed is directly proportional to direction change modulus
- movement speed is inversely proportional to direction change modulus (faster on straights and slower on turns)

Selected environment:
- ROS Noetic - current supported version
- Gazebo11
- Python3
- Docker container with Ubuntu20.04

**Work done by Mikhail Bolotov, ISU number 410969**
