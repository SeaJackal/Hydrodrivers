# Библиотека драйверов и примеров кода подводного аппарата

Библиотека написана на **языке C++** для микроконтроллера STM32,
использует **CMake** в качестве системы сборки, используется библиотека **CMSIS**.

## Структура

В корневой папке лежат папки программных модулей. В них находятся папки **include**
с публичными заголовочными файлами (при наличии драйверов) и папки **example** с кодом примеров,
а также файл **CMakeLists.txt**. Некоторые папки могут содержать папки с реализациями под разные
платформы (например, **f1** и **f4**), а также папки **upper_half** и **lower_half**, содержащие
независимую от платформы логику и платформозависимые драйвера. \
Кроме того, в корневой папке имеются папки **libs** (сторонние библиотеки) и **resources**
с ресурсами сборки.

## Платформы

На данный момент поддерживается два микроконтроллера семейства STM32 - **STM32F103** и **STM32F407**.
Не все модули поддерживают оба контроллера, смотрите предупреждения сборки.

## Сборка

Для инициализации сабмодуля Hydrolib-soft:

```
git submodule update --init --recursive
```

Для полной сборки из корневой директории используйте `DebugF4` для STM32F407 или
`DebugF1` для STM32F103:

```
cmake --preset DebugF4
cmake --build --preset DebugF4
```

Пресеты используют отдельные каталоги, например `build/DebugF4`. Собранные примеры находятся в
`build/<preset>/<module>/example/*.elf`.

### Выборочная сборка

Переменная `HYDRV_BUILD_MODULES` принимает один или несколько модулей `hydrv_*`. CMake автоматически
добавит их транзитивные зависимости. Например, для сборки только `hydrv_tim` и его зависимостей:

```
cmake --preset DebugF4 \
    -B build/DebugF4-hydrv_tim \
    -DHYDRV_BUILD_MODULES=hydrv_tim
cmake --build build/DebugF4-hydrv_tim
```

Для нескольких модулей передайте CMake-список в кавычках:

```
cmake --preset DebugF4 \
    -B build/DebugF4-tim-uart \
    '-DHYDRV_BUILD_MODULES=hydrv_tim;hydrv_uart'
cmake --build build/DebugF4-tim-uart
```

Если `HYDRV_BUILD_MODULES` не задана или равна `all`, собирается полный набор. Неизвестный модуль или модуль,
несовместимый с выбранным `MCU_FAMILY`, вызовет ошибку на этапе конфигурации. Для разных наборов модулей
используйте разные binary directories, чтобы не переиспользовать старое значение из CMake cache.

## Тесты

Сборка и запуск тестов разделены. `BUILD_TESTS` готовит host unit-тесты, а `RUN_TESTS` регистрирует Renode-тесты;
ни один из этих флагов не запускает тесты во время `cmake --build`.

Пример сборки и запуска Renode-теста GPIO:

```
cmake --preset RenodeF4 \
    -B build/RenodeF4-hydrv_gpio \
    -DHYDRV_BUILD_MODULES=hydrv_gpio
cmake --build build/RenodeF4-hydrv_gpio
ctest --test-dir build/RenodeF4-hydrv_gpio --output-on-failure
```

Для host unit-тестов UART:

```
cmake --preset Testing
cmake --build --preset Testing
ctest --preset Testing
```
