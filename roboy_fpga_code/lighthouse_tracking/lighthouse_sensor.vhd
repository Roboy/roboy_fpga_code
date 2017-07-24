library IEEE;
use IEEE.std_logic_1164.all;        
use IEEE.numeric_std.all;
use IEEE.std_logic_signed.all;

-- This module is a state machine with two main states: HIGH and LOW. 
-- In general, the states correspond to the value of the input sensor 
-- signal. To avoid noise, additional filtering is done. The internal 
-- state will change only when the input signal changes and stays constant 
-- for some time.
-- 
-- The module has several internal counters to measure the duration of 
-- pulses. Outputs are updated every time a SWEEP was detected. It is 
-- possible to find out which of the two lighthouses was sweeping the 
-- room depending on the duration between two NSKIP pulses. If this 
-- duration is approximately equal to the phase duration (8333 us), 
-- the active lighthouse didn't change. Otherwise the active lighthouse 
-- has changed. All SKIP pulses are completely ignored.
--
-- Author: Nikita Basargin


entity lighthouse_sensor is 
	port (
		clk : in std_logic;		-- 50 MHz clock
		sensor : in std_logic;	-- sensor INPUT
		duration_nskip_to_sweep: out unsigned(31 downto 0); -- duration NSKIP to SWEEP
		lighthouse_id : out std_logic;	-- which lighthouse emitted the sweep
		axis : out std_logic;				-- sweep x or y axis
		valid : out std_logic;				-- is '1' if (300 * 50 < duration < 8000 * 50)
		combined_data : out unsigned(31 downto 0)
		-- combined data layout:
		-- bit 31    	lighthouse_id
		-- bit 30    	axis
		-- bit 29    	valid
		-- bits 28:0	duration (divide by 50 to get microseconds)
	);
end lighthouse_sensor;


architecture baustein42 of lighthouse_sensor is
	-- INPUT STATES
	-- input signal state: high or low, filtered from noise
	signal sensor_state : std_logic;
	-- counter for noise filtering
	signal sensor_state_switch_counter : unsigned(5 downto 0) := (others => '0');
	
	-- LIGHTHOUSE TIMING
	-- counts time from the last NSKIP pulse (start)
	signal counter_from_nskip_rise : unsigned(31 downto 0) := (others => '0');
	-- counts time from the last signal rising edge
	signal counter_from_last_rise : unsigned(31 downto 0) := (others => '0');
	-- duration from last NSKIP pulse (start) to the last signal rising edge
	signal duration_from_nskip_rise_to_last_rise : unsigned(31 downto 0) := (others => '0');
	
	-- AXIS & LIGHTHOUSE ID
	signal current_axis : std_logic := '0';
	signal current_lighthouse_id : std_logic := '0';
	
begin
	
	process(clk)
	begin
		if rising_edge(clk) then
			
			-- update counters
			counter_from_nskip_rise <= counter_from_nskip_rise + 1;
			counter_from_last_rise <= counter_from_last_rise + 1;
			
			
			if (sensor_state = '0') then 
			
				-- LOW PHASE				
				if (sensor = '0') then
				
					-- KEEP LOW PHASE
					if (sensor_state_switch_counter > 0) then
						sensor_state_switch_counter <= sensor_state_switch_counter - 1;					
					end if;						
					
				else -- sensor = '1'
				
					-- check if state can be switched (no noise)
					if (sensor_state_switch_counter > 25) then		
					
						-- GO TO HIGH PHASE == rising_edge(sensor)
						sensor_state <= '1';
						sensor_state_switch_counter <= "000000";	
						
						-- reset counter that counts high pulse length
						counter_from_last_rise <= "00000000000000000000000000000000";
						-- save duration from last NSKIP start to this rising_edge
						duration_from_nskip_rise_to_last_rise <= counter_from_nskip_rise;
						
					else						
						-- do not change state yet, could be noise
						sensor_state_switch_counter <= sensor_state_switch_counter + 1;
						
					end if;
					
				end if;				
				-- end LOW PHASE
				
			
			else  -- sensor_state = '1'
			
				-- HIGH PHASE		
				if (sensor = '1') then
				
					-- KEEP HIGH PHASE
					if (sensor_state_switch_counter > 0) then
						sensor_state_switch_counter <= sensor_state_switch_counter - 1;					
					end if;	
				
				else -- sensor = '0' 

					-- check if state can be switched (no noise)
					if (sensor_state_switch_counter > 25) then		
					
						-- GO TO LOW PHASE == falling_edge(sensor)
						sensor_state <= '0';
						sensor_state_switch_counter <= "000000";	
						
						-- CHECK PULSE DURATION, we use 50 MHz clock
						if (counter_from_last_rise < 2500) then 
						
							-- last pulse was a SWEEP (max 50 microseconds * 50 clock speed = 2500)		
							-- UPDATE ALL OUTPUTS with previously saved data
							
							-- lighthouse_id
							lighthouse_id <= current_lighthouse_id;
							combined_data(31) <= current_lighthouse_id;
							
							-- axis
							axis <= current_axis;														
							combined_data(30) <= current_axis;
							
							-- valid when (300 * 50) < duration < (8000 * 50)
							if (15000 < duration_from_nskip_rise_to_last_rise) and (duration_from_nskip_rise_to_last_rise < 400000) then
								valid <= '1';
								combined_data(29) <= '1';
							else							
								valid <= '0';
								combined_data(29) <= '0';
							end if;
														
							-- duration from NSKIP to last rising edge is the duration to this SWEEP
							duration_nskip_to_sweep <= duration_from_nskip_rise_to_last_rise;
							combined_data(28 downto 0) <= duration_from_nskip_rise_to_last_rise(28 downto 0);
							
							
						elsif (counter_from_last_rise < 4950) then
						
							-- last pulse was NOT SKIPPING	
							-- save data but publish only on next SWEEP
						
							-- check if active lighthouse has changed
							if (counter_from_nskip_rise < 401650) then
								-- (8333-300) * 50 = 401650
								-- first lighthouse became active
								current_lighthouse_id <= '0'; 

							elsif (counter_from_nskip_rise > 431650) then
								-- (8333+300) * 50 = 431650
								-- second lighthouse became active
								current_lighthouse_id <= '1';  
							
							end if;
							
							-- new NSKIP detected! --> overwrite old counter, time is counted from the last rising edge 
							counter_from_nskip_rise <= counter_from_last_rise + 1;
		
							
							if (counter_from_last_rise < 3350) then
								-- Not skipping, axis = 0, data = 0  
								-- (max 67 microseconds * 50 clock speed = 3350, real time = 62.5 microseconds)
								current_axis <= '0';
								
							elsif (counter_from_last_rise < 3900) then							
								-- Not skipping, axis = 1, data = 0  
								-- (max 78 microseconds * 50 clock speed = 3900, real time = 72.9 microseconds)
								current_axis <= '1';								
								
							elsif (counter_from_last_rise < 4400) then	
								-- Not skipping, axis = 0, data = 1  
								-- (max 88 microseconds * 50 clock speed = 4400, real time = 83.3 microseconds)
								current_axis <= '0';								
								
							else	
								-- Not skipping, axis = 1, data = 1  
								-- (max 99 microseconds * 50 clock speed = 3350, real time = 93.8 microseconds)
								current_axis <= '1';								
								
							end if;
							
						else 						
							-- SKIPPING  (real time = 104 or more microseconds)
							-- ignore this one completely
						
						end if;
						
							
												
						
					else
						-- do not change state yet, could be noise
						sensor_state_switch_counter <= sensor_state_switch_counter + 1;
						
					end if;					
					
				end if;
				
				-- end HIGH PHASE	
				
			end if;
			
			
		end if; -- end rising_edge(clk)
		
	end process;
	
	
	
end architecture;
