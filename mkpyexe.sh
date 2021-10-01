pyinstaller --clean \
	    --p /opt/ros/foxy/lib/python3.8/site-packages \
	    --onefile --collect-all rclpy \
	    --collect-all std_msgs --collect-all rosidl_parser\
	    --collect-all rcl_interfaces\
	    --collect-all builtin_interfaces\
            --hidden-import=rclpy \
            --hidden-import=std_msgs\
            --hidden-import=rosidl_parser\
	    --hidden-import=rcl_interfaces\
	    --hidden-import=builtin_interfaces\
	    --hidden-import=PySide2.QtMultimedia\
	    --add-data "ProjectGui/fonts/digital-7.ttf:fonts/"\
	    --add-data "ProjectGui/images/user-guide.png:images/"\
            --add-data "ProjectGui/main.qml:."\
	    --add-data "/opt/ros/foxy/lib/python3.8/site-packages/rosidl_parser/grammar.lark:rosidl_parser/"\
	    ProjectGui/main.py
