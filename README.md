<h1 align="center">🤖 Line Follower Octoliner + PID (Arduino)</h1>

<p align="center">
  <img src="https://img.shields.io/badge/Arduino-00979D?style=flat&logo=arduino&logoColor=white" />
  <img src="https://img.shields.io/badge/C%2B%2B-00599C?style=flat&logo=c%2B%2B&logoColor=white" />
  <img src="https://img.shields.io/badge/Octoliner-Sensor-orange?style=flat" />
  <img src="https://img.shields.io/badge/PID-Control-blueviolet?style=flat" />
</p>

---

## 📌 Описание

Этот проект — **линейный робот**, основанный на Arduino, с 8‑канальным инфракрасным сенсором Octoliner и PID-регулятором для точного следования линии. Реализованы функции плавного старта‑стопа, разворотов и настройки PID‑коэффициентов.

---

## ⚙️ Аппаратная платформа

- Arduino Uno/Nano
- Сенсорный модуль Pololu / Amperka Octoliner :contentReference[oaicite:1]{index=1}
- Два мотора с драйвером (например, L298N или TB6612FNG)
- Кнопка для запуска (на пине 12)
- Источник питания (LiPo или батарея)

---

## 🧠 Программные компоненты

- Язык: C++ для Arduino IDE
- PID‑алгоритм: пропорционально‑дифференциальная регулировка (P и D)
    ```
    #define Kp 1.291
    #define Kd 20.09
    ```
- Обмен данными с Octoliner через I2C
- Упрощённые функции:
  - `line_white()` и `line_black()` — движение по светлой/тёмной линии
  - `povorotR()` / `povorotL()` — развороты с помощью датчиков
  - Мягкая остановка: `line_stop()`

---

## 📂 Структура репозитория

````

line-follower-octoliner-pid/
├── line\_follower.ino      # Основной скетч
├── README.md             # Документация
├── images/
│   └── robot\_example.jpg # Фото робота (при желании)
└── lib/
└── Octoliner/        # Библиотека sed Octoliner

````

---

## 🧪 Установка и запуск

1. Подключите Octoliner к Arduino (SDA/SCL, питание).
2. Установите библиотеку Octoliner (например, загрузите с GitHub Amperka) :contentReference[oaicite:2]{index=2}.
3. Откройте `line_follower.ino` в Arduino IDE.
4. Убедитесь в корректности подключений моторов и кнопки.
5. Загрузите код на контроллер.
6. Нажмите кнопку — робот начнёт следовать по линии.

---

## 🛠 Настройка PID

В файле скетча:
```
#define Kp 1.291
#define Kd 20.09
```

Для улучшения работы под ваши условия (тип поверхности, освещённость) рекомендуется провести ручную настройку PID‑коэффициентов.


