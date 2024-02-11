cmake_minimum_required(VERSION 3.8)
project(smabo_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ----- ここから追記　--------
find_package(geometry_msgs REQUIRED) # 今回の自作メッセージに必要な（依存関係のある）パッケージ
find_package(std_msgs REQUIRED) # 今回の自作メッセージに必要な（依存関係のある）パッケージ
# メッセージ定義ファイル「IDL」を生成する「rosidl_default_generators」を検索
find_package(rosidl_default_generators REQUIRED) 
# メッセージ定義ファイルの指定
set (msg_files
    "msg/Expression.msg"
    "msg/TwoDimensionPosition.msg"
    "msg/Gyro.msg"
    "msg/Magnetic.msg"
    "msg/Servo.msg"
    "msg/DcMotor.msg"
    )

# IDLの生成
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs # 今回の自作メッセージに必要な（依存関係のある）パッケージ
)
# ----- ここまで ----------------

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
