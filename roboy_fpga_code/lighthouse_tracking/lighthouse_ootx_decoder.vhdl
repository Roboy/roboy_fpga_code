library IEEE;
use IEEE.std_logic_1164.all;        
use IEEE.numeric_std.all;
use IEEE.std_logic_signed.all;

-- This module decodes the ootx frame of up to two lighthouses
--
-- Author: Simon Trendel, Nikita Basargin


entity lighthouse_ootx_decoder is 
	port (
		clk : in std_logic;		-- 50 MHz clock
		reset : in std_logic;
		sensor : in std_logic;	-- sensor INPUT
		sync: out std_logic; -- spikes on valid ootx frame decoding (used for triggering data transmission)
		led : out std_logic_vector(6 downto 0);
		ootx_payload_length_o : out unsigned(15 downto 0);
		ootx_payload_o : out std_logic_vector(271 downto 0);
		ootx_crc32_o : out std_logic_vector(31 downto 0)
	);
end lighthouse_ootx_decoder;

architecture baustein42 of lighthouse_ootx_decoder is
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
	
	-- LIGHTHOUSE ID
	signal current_lighthouse_id : natural;
	
	-- OOTX 
	type ootx_payload is array (1 downto 0) of std_logic_vector(271 downto 0);
	signal ootx_payloads : ootx_payload;	
	constant ootx_preamble : std_logic_vector(17 downto 0) := "000000000000000001";
	type ootx_shift_register is array (1 downto 0) of std_logic_vector(17 downto 0);
	signal ootx_shift_registers : ootx_shift_register;
	type crc32 is array (1 downto 0) of std_logic_vector(31 downto 0);
	signal crc32s : crc32;
	signal data : std_logic_vector(1 downto 0);
	type ootx_state is array(1 downto 0) of std_logic_vector(1 downto 0);
	signal ootx_states : ootx_state;
	type data_counter is array(1 downto 0) of natural;
	signal data_counters : data_counter;
	type bit_counter is array(1 downto 0) of natural;
	signal bit_counters : bit_counter;
	type payload_length is array(1 downto 0) of unsigned(15 downto 0);
	signal payload_lengths : payload_length;
	signal new_data : std_logic;
	
begin
	
	process(clk, reset)
	begin
		if reset = '1' then
			led(0) <= '0';
			led(1) <= '0';
			led(2) <= '0';
			led(3) <= '0';
			ootx_states(0)  <= "00";
			ootx_states(1)  <= "00";
			current_lighthouse_id <= 0; 
		else 
			if rising_edge(clk) then
				sync <= '0';
				if new_data = '1' then
					led(6) <= data(current_lighthouse_id) ;
					if(ootx_states(current_lighthouse_id)  = "00") then 
						-- OOTX preamble detector
						for i in 0 to 16 loop --register shifter
							 ootx_shift_registers(current_lighthouse_id)(i+1) <= ootx_shift_registers(current_lighthouse_id)(i);
						end loop;        

						ootx_shift_registers(current_lighthouse_id) (0) <= data(current_lighthouse_id) ;

						if ootx_shift_registers(current_lighthouse_id)  = ootx_preamble then
							 led(0) <= '1';
							 ootx_states (current_lighthouse_id) <= "01";
							 data_counters (current_lighthouse_id) <= 0;
							 bit_counters(current_lighthouse_id)  <= 0;
						else
							led(0) <= '0';
							led(1) <= '0';
							led(2) <= '0'; 
							led(3) <= '0';
						end if;	
					elsif (ootx_states(current_lighthouse_id)  = "01") then
						-- the following 15 bits are the payload length
						if(data_counters(current_lighthouse_id) < 15) then 
							payload_lengths(current_lighthouse_id) (data_counters(current_lighthouse_id) ) <= data(current_lighthouse_id) ;
							data_counters(current_lighthouse_id)  <= data_counters(current_lighthouse_id)  + 1;
						else
							if (data(current_lighthouse_id)  = '1') then -- we want the sync bit
								led(1) <= '1';
								data_counters(current_lighthouse_id)  <= 0;
								bit_counters(current_lighthouse_id)  <= 0;
								ootx_states(current_lighthouse_id)  <= "10";
								payload_lengths(current_lighthouse_id) <= shift_right(payload_lengths(current_lighthouse_id),1) +
																								shift_right(payload_lengths(current_lighthouse_id),1) mod 2;
								ootx_payload_length_o<= payload_lengths(current_lighthouse_id);
							else -- go back to preamble detection
								ootx_states(current_lighthouse_id)  <= "00";
							end if;
						end if;
					elsif (ootx_states(current_lighthouse_id)  = "10") then
						-- the following payload_length bytes are the payload
						if(data_counters(current_lighthouse_id)  < shift_left(payload_lengths(current_lighthouse_id),3)) then 
							if((bit_counters(current_lighthouse_id)+1) mod 17 = 0) then
								if (data(current_lighthouse_id)  = '0') then -- we want the sync bit, so if it's not there go to preamble detection
									ootx_states(current_lighthouse_id)  <= "00";
								end if;
							else -- write the bit to payload 
								ootx_payloads(current_lighthouse_id)  (data_counters(current_lighthouse_id)) <= data(current_lighthouse_id) ;
								data_counters(current_lighthouse_id)  <= data_counters(current_lighthouse_id)  + 1;
							end if;
							bit_counters(current_lighthouse_id)  <= bit_counters(current_lighthouse_id)  + 1;
						else 
							data_counters(current_lighthouse_id)  <= 0;
							bit_counters(current_lighthouse_id)  <= 0;
							ootx_states(current_lighthouse_id)  <= "11";
							ootx_payload_o<= ootx_payloads(current_lighthouse_id);
							led(2) <= '1';
						end if;
					elsif (ootx_states(current_lighthouse_id)  = "11") then
						-- the following 32 bit are the crc checksum of the payload
						if((bit_counters(current_lighthouse_id)+1) mod 17 = 0) then
							if (data(current_lighthouse_id)  = '0') then -- we want the sync bit, so if it's not there go to preamble detection
								ootx_states(current_lighthouse_id)  <= "00";
							end if;
						else
							if(data_counters(current_lighthouse_id)  < 32) then
								crc32s(current_lighthouse_id) (data_counters(current_lighthouse_id)) <= data(current_lighthouse_id) ;
								data_counters(current_lighthouse_id)  <= data_counters(current_lighthouse_id)  + 1;
							else
								ootx_states(current_lighthouse_id)  <= "00";
								ootx_crc32_o <= crc32s(current_lighthouse_id);
								sync <= '1';
								led(3) <= '1';
							end if;
						end if;
						bit_counters(current_lighthouse_id)  <= bit_counters(current_lighthouse_id)  + 1;	
					end if;
				end if;
				
				new_data <= '0';
			 
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
								-- duration from NSKIP to last rising edge is the duration to this SWEEP
								
							elsif (counter_from_last_rise < 4950) then
							
								-- last pulse was NOT SKIPPING	
								-- save data but publish only on next SWEEP
							
								-- check if active lighthouse has changed
								if (counter_from_nskip_rise < 401650) then
									-- (8333-300) * 50 = 401650
									-- first lighthouse became active
--									current_lighthouse_id <= 0; 
--									led(6) <= '0'; 

								elsif (counter_from_nskip_rise > 431650) then
									-- (8333+300) * 50 = 431650
									-- second lighthouse became active
--									current_lighthouse_id<= 1;  
--									led(6) <= '1'; 
								end if;
								
								-- new NSKIP detected! --> overwrite old counter, time is counted from the last rising edge 
								counter_from_nskip_rise <= counter_from_last_rise + 1;
			
								if (counter_from_last_rise < 3350) then
									-- Not skipping, axis = 0, data = 0  
									-- (max 67 microseconds * 50 clock speed = 3350, real time = 62.5 microseconds)
									data(current_lighthouse_id) <= '0';
									
								elsif (counter_from_last_rise < 3900) then							
									-- Not skipping, axis = 1, data = 0  
									-- (max 78 microseconds * 50 clock speed = 3900, real time = 72.9 microseconds)
									data(current_lighthouse_id) <= '0';							
									
								elsif (counter_from_last_rise < 4400) then	
									-- Not skipping, axis = 0, data = 1  
									-- (max 88 microseconds * 50 clock speed = 4400, real time = 83.3 microseconds)
									data(current_lighthouse_id) <= '1';
									
								else	
									-- Not skipping, axis = 1, data = 1  
									-- (max 99 microseconds * 50 clock speed = 3350, real time = 93.8 microseconds)
									data(current_lighthouse_id) <= '1';
								end if;							
								new_data <= '1';
							else 						
								-- SKIPPING  (real time = 104 or more microseconds)
								if (counter_from_last_rise < 5450) then
									-- skipping, axis = 0, data = 0  
									-- (max 109 microseconds * 50 clock speed = 5450, real time = 104 microseconds)
									data(current_lighthouse_id) <= '0';
									
								elsif (counter_from_last_rise < 6000) then							
									-- skipping, axis = 1, data = 0  
									-- (max 120 microseconds * 50 clock speed = 6000, real time = 115 microseconds)
									data(current_lighthouse_id) <= '0';							
									
								elsif (counter_from_last_rise < 6500) then	
									-- skipping, axis = 0, data = 1  
									-- (max 130 microseconds * 50 clock speed = 6500, real time = 125 microseconds)
									data(current_lighthouse_id) <= '1';
									
								else	
									-- skipping, axis = 1, data = 1  
									-- (max 140 microseconds * 50 clock speed = 7000, real time = 135 microseconds)
									data(current_lighthouse_id) <= '1';
									
								end if;
								new_data <= '1';
							end if;
							
							
						else
							-- do not change state yet, could be noise
							sensor_state_switch_counter <= sensor_state_switch_counter + 1;
							
						end if;					
						
					end if;
					
					-- end HIGH PHASE	
					
				end if;
					
			end if; -- end rising_edge(clk)
		end if; -- end not reset
		
	end process;
	
	
	
end architecture;
