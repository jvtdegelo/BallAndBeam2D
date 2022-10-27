library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pid is
    port (
        clock            : in  std_logic;
		equilibrio		 : in  std_logic_vector (9 downto 0);
        entrada_sensor   : in  std_logic_vector (9 downto 0); 
        saida_servo      : out std_logic_vector (9 downto 0) 
    );
end pid;
architecture behavioral of pid is
    type steps is (Reset,
			CalculateNewError,
			CalculatePID,
			DivideKg,
			Write2DAC,                              
			SOverload,
			ConvDac);	                             
    
    signal state,next_state : steps := Reset;     
    signal Kp : integer := 10;		--proportional constant
    signal Kd : integer :=20;		--differential constant
    signal Ki : integer :=1;		--integral constant
    signal output : integer := 1;	
    signal inter: integer := 0;		
    signal SetVal : integer := 256; 
    signal sAdc : integer := 0 ;	--stores the integer converted value of the ADC input
    signal Error: integer := 0;		--Stores the deviation of the input from the set point
    signal p,i,d : integer := 0;	--Contain the proportional, derivative and integral errors respectively
    signal DacDataCarrier : std_logic_vector (9 downto 0); --contains the binary converted value to be output to the DAC
    
begin
	setVal <= to_integer(unsigned(equilibrio));
process(clock,state)		--sensitive to Clock and current state
      variable output_old : integer := 0;   
      variable error_old : integer := 0;
     BEGIN	 
         IF clock'EVENT AND clock='1' THEN  
				state <= next_state;
         END IF;
         case state is
		 when Reset =>
			sAdc <= to_integer(unsigned(entrada_sensor));  --Get the input for PID
			next_state <= CalculateNewError;
			error_old := Error;  --Capture old error
			output_old := output;    --Capture old PID output
			
		  when CalculateNewError =>  
			next_state <= CalculatePID;
			inter <= (SetVal-sAdc); --Calculate Error
			Error <= to_integer(to_unsigned(inter,32));
		  
		  when CalculatePID =>
			next_state <= DivideKg;
			p <= Kp*(Error);              --Calculate PID 
			i <= Ki*(Error+error_old);
			d <= Kd *(Error-error_old);                     
				
		  when DivideKg =>
			next_state <= SOverload;
			output <=  output_old+(p+i+d)/2048; --Calculate new output (/2048 to scale the output correctly)
		  
		  when SOverload =>
			next_state <=ConvDac;	--done to keep output within 16 bit range
			if output > 512 then
				 output <= 512 ;
			end if;     
			if output < 1 then 
				 output <= 1;
			end if;
				
		  when ConvDac =>        		--Send the output to port
			DacDataCarrier <= std_logic_vector(to_unsigned(output ,9));
			next_state <=Write2DAC;
			
		  when Write2DAC =>				--send output to the DAC
			next_state <= Reset;
			saida_servo <= DacDataCarrier;
	 end case;

                        
end process;	--end of process
end behavioral;		--end of Architecture