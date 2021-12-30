------------------------------------------------------------
-------- P_Brake_StateMachine ArduPilot Lua script ---------
--                                                        --
-- 		This script is a function that detects when       --
-- 	the vehicle reaches the tree planting point and gives --
--	the robot a signal in place to return.                --
--                                                        --
--------------------- Teddy -- Nov 2021 --------------------
--https://ardupilot.org/rover/docs/common-lua-scripts.html--
------------------------------------------------------------

--------    USER EDITABLE GLOBALS  --------
local ROCKER_MAX_PWM 	 = 1920 --摇杆最大值
local ROCKER_MID_PWM 	 = 1500 --摇杆悬空值
local ROCKER_MIN_PWM 	 = 1100 --摇杆最小值
local SET_OUTPUT_PWM_ON  = 1 	--给PWM引脚输出高电平
local SET_OUTPUT_PWM_OFF = 0 	--给PWM引脚输出低电平
local ARRIVE = false 	     	--到位信号默认为false
-------- END USER EDITABLE GLOBALS --------

--------    "CONSTANTS"   --------
local ROVER_MODE_MANUAL  = 0
local ROVER_MODE_HOLD 	 = 4
local ROVER_MODE_AUTO 	 = 10
local RELAY_NUM 		 = 0

local wp_index
local wp_distance				-- 距离
local wp_bearing				-- 方向姿态
local wp_error
local wp_max_distance 	 = 0
local last_log_ms 		 = millis()

--local analog_in = analog:channel()   		 --读取GPIO的模拟值
local digital_in 		 = digital:channel() --读取GPIO的数字值

-- load a input pwm pin --
local pwm_in 			 = PWMSource()
local pwm_in_fail 		 = PWMSource()

-------- END "CONSTANTS"  --------


------------------------------------------------------------------
--这是由于中断的处理方式，因为它们使用相同的中断（on Cube）,AUX 1和6不能同时处理.
------------------------------------------------------------------
--	sen_text(severity,text)		--
--	Severity		text 		--
--	0				Emergency	--
--	1				Alert		--
-- 	2				Critical	--
--  3				Error		--
--	4				Warning		--
--	5				Notice		--
--	6				Info 		--
-------------------------------------------------------------------
if not pwm_in:set_pin(50) then		  --AUX 1
	gcs:send_text(0,"Failed to setup PWM in pin 50 (AUX 1)")
end

if not pwm_in_fail:set_pin(55) then   	--AUX6
	gcs:send_text(0,"Failed to setup PWM in pin 55 (AUX 6)")
end

gpio:pinMode(51,0) 	--set AUX 2 to input mode, gpio:pinMode(51,1)would be output mode  ---robot to pixhawk
gpio:pinMode(52,1)  --set AUX 3 to output mode										   ---pixhawk to robot

gpio:write(52,SET_OUTPUT_PWM_OFF)		--给AUX3写入低电平


-----------------------------------------------------------------------
-- perform_robot_actions()是与ROBOT通信，确保ROBOT种树动作已经完成，并前往下一个点 
-----------------------------------------------------------------------
function perform_robot_actions()
	local gpio_data_aux2				--局部变量
	gpio_data_aux2 = gpio:read(51)		--读取AUX2的值赋给gpio_data
	if gpio_data_aux2 then
		gcs:send_text(6,"ROBOT种值完成!")
		set_mode(ROVER_MODE_AUTO)

	else
		gcs:send_text(3,"ROBOT还在工作!")

	end

	return perform_robot_actions,1000

end


-------------------------------------------------------------------------
-- 存储当前位置 --
-------------------------------------------------------------------------
function on_wp_change(index,distance)
	wp_index = index
	wp_distance = distance
	wp_distance_max = distance
	gcs:send_text(0,string.format("NEW WP: idx=%d, dist=%.01fm",index,distance))
end


-------------------------------------------------------------------------
-- refresh_wp_info() 得到当前的位置信息\并且跟新位置信息
-------------------------------------------------------------------------
function refresh_wp_info()
	local index = mission:get_current_nav_index()
	local distance = vehicle:get_wp_distance_m()
	local bearing = vehicle:get_wp_bearing_deg()
	local error = vehicle:get_wp_crosstrack_error_m()

	if index ~= nil and index ~= wp_index and distance ~= nil then
		on_wp_change(index,distance)
	end

-----------------------------------------------------
------- 存储当前位置、为下一次跟新位置做比较 -----------
-----------------------------------------------------
	if index ~= nil and distance ~= nil and bearing ~= nil and error ~= nil then
		wp_index = index
		wp_bearing = bearing
		wp_distance = distance
		wp_error = error

		wp_distance_max = math.max(wp_distance_max,wp_distance)
		set_mode(ROVER_MODE_HOLD) 			--如果车到达了种树点，停车
		gpio:write(52,SET_OUTPUT_PWM_ON)	--给AUX3写入高电平
		perform_robot_actions()				--调用与机器人通信功能函数

	end

end

-------------------------------------------------------------------------
-- 打印消息到地面站 --
-------------------------------------------------------------------------
function log_wp_info(index, distance, bearing, error)
    if index ~= nil and distance ~= nil and bearing ~= nil and error ~= nil then
        local perc = wp_distance_max > 0 and 100-(100*(distance/wp_distance_max)) or 0
        gcs:send_text(0, string.format("WP: %d, %.01fm (%.01f%%), b: %d°, xe:%.01fm", index, distance, perc, math.floor(bearing+0.5), error))
    end
end

-------------------------------------------------------------------------
-- 开始函数调用
-------------------------------------------------------------------------
function update()
	if vehicle:get_mode() == ROVER_MODE_AUTO then
		refresh_wp_info()
    	log_wp_info(wp_index, wp_distance, wp_bearing, wp_error)
    	return update, 1000 				-- 1Hz
	end
end


return update(), 1000 						-- start with a 1 sec delay
