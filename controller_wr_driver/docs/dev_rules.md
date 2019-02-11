Привет, разработчик!  
Здесь будут перечислены некоторые правила, которых стоит придерживаться для создания единого стиля разработки =)

* Оформление функций в соответствии со стилем `CamelCase` [`breakSensorIsPressed()`] или через `snake_case` [`break_sensor_is_pressed()`].

* Комментарии к функциям устанавливаются в соответствии с 
[Doxygen стилем](https://www.rosettacommons.org/docs/latest/development_documentation/tutorials/doxygen-tips) / [Еще](https://www.stack.nl/~dimitri/doxygen/manual/commands.html)  
Например
```c++
/*
 * @brief       This function makes something good
 * @param [in]  val  Motor power value [0-100]
 */

```
или
```c++
/**
 * Brief description of method
 * @pre
 *    Any preconditions that this method requires
 * @post
 *    Any postconditions that result because of this method
 * @param [0, n-1] paramName
 *    Brief description of parameter
 * @return
 *    Brief description of return value
 * @throw
 *    An EXPLICIT throw specification - including which guarantee it offers (basic, strong, nothrow)
 *    and WHY the various exceptions may be thrown.
 */
```

* Комментарии к переменным можно снабжать однострочными комментариями, желательно на строке, предшествующей определению переменной
```c++
/* Encoder tick count, should be limited by range [-1000; 1000] */
int16_t ticks = 0;

```

* Каждый модуль должен снабжаться заголовочным файлом с набором данных, которые отображают, что можно использовать в других модулях (открытые прототипы и структуры)

```c++
/* Header content */
#ifndef HEADER_NAME_H_
#define HEADER_NAME_H_

typedef struct
{
    /* Absolute value of accelerometer range */
    uint32_t accel_range;

    /* Absolute value of gyroscope range */
    uint32_t gyro_range;

} imu_conf_t;

/* 32 bits - *100 multiplier encoded float value */
/* 3245 <- 32.45 accel value */
typedef int32_t imu_accel_val_t;

/**
 * @brief
 *      Initialize and start IMU driver
 * @param   conf
 *      Pointer to configuration structure    
 * @return
 *      EOK     - ok
 *      EFAULT  - failed to initialize
 */
int imu_start(imu_conf_t *conf);

/**
 * @brief       
 *      Get IMU acceleration value by axis X
 * @param   out_x_val   
 *      Referenced value of X acceleration to be written
 * @return      
 *      EOK     - ok
 *      EFAULT  - failed calculation
 *      EIO     - reading failed
 */
int imu_get_accel_x(imu_accel_val_t *out_x_val);

...

#endif // HEADER_NAME_H_
```

* Отступы (indents) должны иметь размер 4 пробела (не табуляция) [редакторы могут заменять нажатие Tab на необходимый отступ]

* Определение типов задается по шаблону `<название типа>_t`, например, `breakPower_t`

* Любой разрабатываемый модуль должен проходить стадию разработки, тестирования и добавления в основную ветку (develop). В ветку мастер (master) добавляются только версии со 100% рабочими изменениями при согласии всех разработчиков.

* Тестирование должно проводиться в отдельно разработанном приложении с минимальным количеством кода при максимально использовании модуля. Постарайтесь как можно разносторонне проверить модуль =). Для примера, проверка сенсора может производиться путем периодического чтения данных и вывода в последовательный порт форматированным выводом. Само тестирование предполагает изменение условий сенсора и проверка вывода.


* В комментарии к коммиту указывается название ветки или название разрабатываемого модуля. Для общих исправлений используется тэг `[common]`.
```bash
git commit -am '[frontWheel]: New prototype for: int direction_get_value(dirVal_t *out_value);'

```

* При закрытии issue необходимо указывать коммент, в котором были сделаны последние изменения по теме issue.
```bash
git commit -am '[directionDriver]: Fixed #34'

```
* При разрешении issue необходимо присваивать работу над ним себе (если оно ранее уже не было присвоено), тем самым вы обозначите, что вы берётесь зе его исправление. Кроме того, по возможности необходимо классифицировать его с помощью лейбла:
- enchancement (нововведедние / предложение)
- bug
- help wanted 
- duplicate
- good first issue
- invalid

---
Просто некоторые заметки:

* Хороший гайд по ведению веток: http://nvie.com/posts/a-successful-git-branching-model

* Для добавления собственного модуля необходимо добавить указания `.c` файлов в project.mk.
