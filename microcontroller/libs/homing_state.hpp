enum class HomingState {
          HOMING_NOT_STARTED,
          MOVE_UNTIL_NO_SENSOR_READ,
          FIND_FIRST_SENSOR_READ,
          READ_UNITIL_SENSOR_NO_LONGER_SENSES,
          REVERSE_DIRECTION_HALF_THE_STEPS,
          HOMING_FINISHED
      };