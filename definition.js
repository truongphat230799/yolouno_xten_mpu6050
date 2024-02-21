var BlockColor = "#2ECC71";

Blockly.Blocks['angle_sensor_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "angle_sensor_init",
        "message0": "khởi tạo cảm biến góc %1 calib %2 mẫu",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "type",
            "options": [
              ["MPU6050", "MPU6050"],
              ["MPU6050 + HMC5883L", "MPU6050_HMC5883L"],
              ["MPU9250", "MPU9250"]
            ]
          },
          {
            type: "input_value",
            check: "Number",
            value: 100,
            name: "samples",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": BlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

// Angle sensor

Blockly.Python["angle_sensor_init"] = function (block) {
  var type = block.getFieldValue("type");
  var samples = Blockly.Python.valueToCode(block, 'samples', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  if (type == "MPU6050") {
    Blockly.Python.definitions_['import_mpu6050'] = 'from mpu6050 import MPU6050';
    Blockly.Python.definitions_['init_mpu6050'] = 'imu = MPU6050()';
  }
  
  if (type == "MPU6050_HMC5883L") {
    Blockly.Python.definitions_['import_mpu6050_hmc5883l'] = 'from mpu6050_hmc5883l import MPU6050_HMC5883L';
    Blockly.Python.definitions_['init_mpu6050_hmc5883l'] = 'imu = MPU6050_HMC5883L()';
  }

  if (type == "MPU9250") {
    Blockly.Python.definitions_['import_mpu9250'] = 'from mpu9250 import MPU9250';  
    Blockly.Python.definitions_['init_mpu9250'] = 'imu = MPU9250()';
  }
  
  Blockly.Python.definitions_['import_angle_sensor'] = 'from angle_sensor import AngleSensor';
  Blockly.Python.definitions_['init_angle_sensor'] = 'angle_sensor = AngleSensor(imu)';

  var code = 'await angle_sensor.calibrate(' + samples + ')\n' + 
    'await angle_sensor.start()\n';
    
  return code;
};

Blockly.Blocks["angle_sensor_get"] = {
  init: function () {
    this.jsonInit({
      colour: BlockColor,
      tooltip: "",
      message0: "đọc %1 cảm biến góc",
      args0: [
        {
          type: "field_dropdown",
          name: "AXIS",
          options: [
            ["heading", "heading"],
            ["pitch", "pitch"],
            ["roll", "roll"],
            ["tất cả thông số", "print_data()"],
          ],
        }
      ],
      output: "Number",
      helpUrl: ""
    });
  },
};

Blockly.Python["angle_sensor_get"] = function (block) {
  var axis = block.getFieldValue("AXIS");
  Blockly.Python.definitions_['setup_angle_sensor'] = 'create_task(angle_sensor.run())';
  // TODO: Assemble Python into code variable.
  var code = "angle_sensor." + axis;
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Blocks["angle_sensor_get_imu"] = {
  init: function () {
    this.jsonInit({
      colour: BlockColor,
      tooltip: "",
      message0: "đọc %2 cảm biến %1",
      args0: [
        {
          type: "field_dropdown",
          name: "SENSOR",
          options: [
            ["accelerometer", "accel"],
            ["gyroscope", "gyro"],
            ["magnetometer", "mag"],
          ],
        },
        {
          type: "field_dropdown",
          name: "AXIS",
          options: [
            ["X", "x"],
            ["Y", "y"],
            ["Z", "z"],
          ],
        }
      ],
      output: "Number",
      helpUrl: ""
    });
  },
};

Blockly.Python["angle_sensor_get_imu"] = function (block) {
  var sensor = block.getFieldValue("SENSOR");
  var axis = block.getFieldValue("AXIS");
  // TODO: Assemble Python into code variable.
  var code = "imu." + sensor + "." + axis + "";
  return [code, Blockly.Python.ORDER_NONE];
};
