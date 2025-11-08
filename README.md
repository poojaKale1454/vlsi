# vlsi
# MAIN VHDL MODEL (MVM)-ALU EXP1 
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL; use IEEE.NUMERIC_STD.ALL;

entity ALU_4bit is
Port ( A : in STD_LOGIC_VECTOR (3 downto 0); B : in STD_LOGIC_VECTOR (3 downto 0);
F : in STD_LOGIC_VECTOR (2 downto 0); Y : out STD_LOGIC_VECTOR (3 downto 0); C_B : out STD_LOGIC
);
end ALU_4bit;

architecture ALU_4bit_arch of ALU_4bit is

signal result:STD_LOGIC_VECTOR(4 downto 0):="00000"; begin
process(A,B,F) begin
CASE F IS

when "000" =>
result <= '0' & (A AND B);

when "001" =>
result <= '0' & (A NAND B);

when "010" =>
result <= '0' & (A OR B);

when "011" =>
result <= '0' & (A XOR B);

when "100" =>
result <= '0' & (A XNOR B);
when "101" =>
result <= '0' & (A NOR B);

when "110" =>
result <= ('0' & A)+('0' & B);

 
when others => if A < B then
result <= '0' & (NOT B); result <= result+1;
result <= ('0' & A) + result; result <= (NOT result) +1;
result <= (NOT(('0' & A) + ('0' &(NOT B)) + 1))+1;
else
result <=('0' & A)-('0' & B); end if ;
end CASE; end process;
Y <= result(3 downto 0); C_B <= result(4);

end ALU_4bit_arch;

#TESTBENCH VHDL MODEL (TVM)

LIBRARY ieee;
USE ieee.std_logic_1164.ALL; USE ieee.std_logic_unsigned.ALL;

ENTITY ALU_4bit_tb IS END ALU_4bit_tb;

ARCHITECTURE behavior OF ALU_4bit_tb IS
-- Component Declaration for the Unit Under Test (UUT) COMPONENT ALU_4bit
PORT(
A : IN std_logic_vector(3 downto 0);
B : IN std_logic_vector(3 downto 0); F : IN std_logic_vector(2 downto 0);
Y : OUT std_logic_vector(3 downto 0);
C_B : OUT std_logic
);
END COMPONENT;

--Inputs
signal A : std_logic_vector(3 downto 0) := "0010"; signal B : std_logic_vector(3 downto 0) := "1111";
signal F : std_logic_vector(2 downto 0) := (others => '1');
--Outputs
signal Y : std_logic_vector(3 downto 0); signal C_B : std_logic;
-- No clocks detected in port list. Replace <clock> below with
-- appropriate port name BEGIN
uut: ALU_4bit PORT MAP (
A => A,
B => B, F => F, Y => Y,
C_B => C_B
);
stim_proc_F: process begin
F <= F + 1;
wait for 25 ns;

end process;
END;
-------------------------------------------------------------------------------------------------------------------------------------------------------------------
#MAIN VHDL MODEL ( MVM ) -
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL; use IEEE.NUMERIC_STD.ALL;


entity UNI_Shift_Register is Port ( rst : in STD_LOGIC;
clk : in STD_LOGIC; Sin : in STD_LOGIC;
mode : in STD_LOGIC_VECTOR (1 downto 0); Pin : in STD_LOGIC_VECTOR (3 downto 0); Sout : out STD_LOGIC;
Pout : out STD_LOGIC_VECTOR (3 downto 0)
);
end UNI_Shift_Register;

architecture UNI_Shift_Register_arch of UNI_Shift_Register is SIGNAL temp : STD_LOGIC_VECTOR (3 downto 0):="0000";
begin
PROCESS(rst, clk, mode, Sin, Pin) BEGIN
IF rst = '1' THEN
Pout <= "0000";
Sout <= '0';

ELSIF FALLING_EDGE(clk) THEN

CASE mode IS
WHEN "00" =>
temp(3 downto 1) <= temp(2 downto 0); temp(0) <= Sin;

Sout <= temp(3); Pout <= "0000";

WHEN "01" =>
temp(3 downto 1) <= temp(2 downto 0); temp(0) <= Sin;
Pout <= temp; Sout <= '0';
WHEN "10" =>
temp <= Pin; Sout <= temp(3);
temp(3 downto 1) <= temp(2 downto 0);

 
Pout <= "0000";

WHEN OTHERS =>
Pout <= Pin; Sout <= '0';

END CASE;
END IF;
END PROCESS;

end UNI_Shift_Register_arch;

#TESTBENCH VHDL MODEL ( TVM)
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

ENTITY UNI_Shift_Register_tb IS END UNI_Shift_Register_tb;

ARCHITECTURE behavior OF UNI_Shift_Register_tb IS
-- Component Declaration for the Unit Under Test (UUT) COMPONENT UNI_Shift_Register
PORT(
rst : IN std_logic; clk : IN std_logic;
mode : IN std_logic_vector(1 downto 0); Sin : IN std_logic;
Pin : IN std_logic_vector(3 downto 0); Sout : OUT std_logic;
Pout : OUT std_logic_vector(3 downto 0)
);
END COMPONENT;

 
--Inputs
signal rst : std_logic := '0'; signal clk : std_logic := '1';
signal mode : std_logic_vector(1 downto 0) := (others => '0'); signal Sin : std_logic := '0';
signal Pin : std_logic_vector(3 downto 0) := "1010";

--Outputs signal Sout : std_logic;
signal Pout : std_logic_vector(3 downto 0);

-- Clock period definitions
constant clk_period : time := 10 ns; BEGIN
-- Instantiate the Unit Under Test (UUT) uut: UNI_Shift_Register PORT MAP (
rst => rst, clk => clk,
mode => mode, Sin => Sin,
Pin => Pin, Sout => Sout, Pout => Pout
);

-- Clock process definitions clk_process :process
begin

clk<=NOT(clk);
wait for clk_period/2 ;
end process;

-- Stimulus process stim_proc_mode: process begin
mode<="00";
wait for 80 ns;

mode<="01"; wait for 50 ns;
mode<="10"; wait for 50 ns;

mode<="11"; wait for 20 ns;
end process;

 
stim_proc_Sin:process begin
wait for 10 ns;
Sin<='1';
wait for 10 ns;

Sin<='0';
wait for 10 ns;

Sin<='1';
wait for 10 ns;

Sin<='0';
wait for 10 ns;

Sin<= '0';
wait for 40 ns;

Sin<='1';
wait for 10 ns;

Sin<='0';
wait for 10 ns;
Sin<='1';
wait for 10 ns;

Sin<='0';
wait for 10 ns;

Sin<= '0';
wait ;
end process;
END;
 
stim_proc_rst:process begin
wait for 122.5 ns; rst<='1';
wait for 5 ns; rst<='0';
wait ;
end process;

-----------------------------------------------------------------------------------------------------------------------------------------------------------------
#MAIN VHDL MODEL ( MVM ) - FIFO

library IEEE;
use IEEE.STD_LOGIC_1164.ALL; use IEEE.NUMERIC_STD.ALL;
entity fifo is
generic (depth : integer := 16); --depth of fifo port ( clk : in std_logic;
reset : in std_logic;
enr : in std_logic; --enable read,should be '0' when not in use. enw : in std_logic;	--enable write,should be '0' when not in use. data_in : in std_logic_vector (7 downto 0);	--input data data_out : out std_logic_vector(7 downto 0);		--output data fifo_empty : out std_logic;	--set as '1' when the queue is empty fifo_full : out std_logic	--set as '1' when the queue is full
);
end fifo;

architecture fifo_arch of fifo is

type memory_type is array (0 to depth-1) of std_logic_vector(7 downto 0);
signal memory : memory_type :=(others => (others => '0')); --memory for queue. signal readptr,writeptr : integer := 0; --read and write pointers.
signal empty,full : std_logic := '0'; begin
fifo_empty <= empty; fifo_full <= full;

process(Clk,reset)
--this is the number of elements stored in fifo at a time.
--this variable is used to decide whether the fifo is empty or full. variable num_elem : integer := 0;
begin
if(reset = '1') then

--	for i in 0 to depth-1 loop
--
--	memory(i)<=(others=>'0');
--	end loop;
memory <= (others => (others=> '0'));

data_out <= (others => '0'); empty <= '1';
full <= '0';
readptr <= 0;
writeptr <= 0;
num_elem := 0; elsif(rising_edge(Clk)) then
if(enr = '1' and empty = '0') then --read

 
data_out <= memory(readptr); readptr <= readptr + 1; num_elem := num_elem-1;
end if;
if(enw ='1' and full = '0') then	--write memory(writeptr) <= data_in; writeptr <= writeptr + 1;
num_elem := num_elem+1; end if;
--rolling over of the indices.
if(readptr = depth-1) then	--resetting read pointer. readptr <= 0;
end if;
if(writeptr = depth-1) then	--resetting write pointer. writeptr <= 0;
end if;
--setting empty and full flags. if(num_elem = 0) then
empty <= '1'; else
empty <= '0'; end if;
if(num_elem = depth) then full <= '1';
else
full <= '0'; end if;
end if;
end process; end fifo_arch;

#TESTBENCH VHDL MODEL ( TVM )


LIBRARY ieee;
USE ieee.std_logic_1164.ALL; USE ieee.std_logic_arith.ALL;
ENTITY fifo_tb IS END fifo_tb;

ARCHITECTURE behavior OF fifo_tb IS
--Inputs and outputs
signal Clk,reset,enr,enw,empty,full : std_logic := '0';
signal data_in,data_out : std_logic_vector(7 downto 0) := (others => '0');
--temporary signals signal i : integer := 0;
-- Clock period definitions
constant Clk_period : time := 10 ns;
constant depth : integer := 16; --specify depth of fifo here.

BEGIN

-- Instantiate the Unit Under Test (UUT)
uut: entity work.fifo generic map(depth => depth) PORT MAP (clk,reset,enr,enw,data_in,data_out,empty,full);

-- Clock process definitions Clk_process :process begin
Clk <= '0';
wait for Clk_period/2; Clk <= '1';
wait for Clk_period/2; end process;
-- Stimulus process stim_proc: process begin
reset <= '1'; --apply reset for one clock cycle. wait for clk_period;
reset <= '0';
wait for clk_period*3; --wait for 3 clock periods(simply) enw <= '1';	enr <= '0';	--write 10 values to fifo.
for i in 1 to 10 loop
Data_In <= conv_std_logic_vector(i,8); wait for clk_period;
end loop;
enw <= '0';	enr <= '1';	--read 4 values from fifo. wait for clk_period*4;
enw <= '0';	enr <= '0';
wait for clk_period*10; --wait for some clock cycles.

 
enw <= '1';	enr <= '0';	--write 10 values to fifo. for i in 11 to 20 loop
Data_In <= conv_std_logic_vector(i,8); wait for clk_period;
end loop;
enw <= '0';	enr <= '0';
wait for clk_period*10; --wait for some clock cycles. enw <= '0';	enr <= '1';	--read 4 values from fifo. wait for clk_period*4;
enw <= '0';	enr <= '0'; wait for clk_period;
enw <= '0';	enr <= '1';	--read 4 values from fifo. wait for clk_period*8;
enw <= '0';	enr <= '0'; wait for clk_period;
enw <= '0';	enr <= '1';	--read 8 values from fifo. wait for clk_period*4;
enw <= '0';	enr <= '0'; wait for clk_period;
enw <= '0';	enr <= '1';	--read 4 values from fifo. wait for clk_period*4;
enw <= '0';	enr <= '0'; wait;
end process;
END;

-------------------------------------------------------------------------------------------------------------------------------------------------------------------
#Keypad VHDL code
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
entity Keypad_new is
    Port ( rows : in  STD_LOGIC_VECTOR (3 downto 0);
           coloumn : in  STD_LOGIC_VECTOR (3 downto 0);
           y : out  STD_LOGIC_VECTOR (7 downto 0));
end Keypad_new;

architecture Behavioral of Keypad_new is

begin
process(rows,coloumn)
begin
if (rows="0001")then
  if (coloumn="0001")then
  y<="00111111";
  end if ;

if (coloumn="0010")then
y<="00000110";
end if ;

if (coloumn="0100")then
y<="01011011";
end if ;

if (coloumn="1000")then
y<="01001111";
end if ;

end if ;

if (rows="0010")then
if (coloumn="0001")then
y<="01100110";
end if ;

if (coloumn="0010")then
y<="01101101";
end if ;

if (coloumn="0100")then
y<="01111101";
end if ;

if (coloumn="1000")then
y<="00000111";
end if ;

end if ;

if (rows="0100")then
if (coloumn="0001")then
y<="01111111";
end if ;

if (coloumn="0010")then
y<="01100111";
end if ;

if (coloumn="0100")then
y<="01110111";
end if ;

if (coloumn="1000")then
y<="01111100";
end if ;

end if ;
if (rows="1000")then
if (coloumn="0001")then
y<="00111001";
end if ;

if (coloumn="0010")then
y<="01011110";
end if ;

if (coloumn="0100")then
y<="01111001";
end if ;

if (coloumn="1000")then
y<="01110001";
end if ;

end if ;
end process;
end Behavioral;

#Test Bench
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY key IS
END key;
 
ARCHITECTURE behavior OF key IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT Keypad_new
    PORT(
         rows : IN  std_logic_vector(3 downto 0);
         coloumn : IN  std_logic_vector(3 downto 0);
         y : OUT  std_logic_vector(7 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal rows : std_logic_vector(3 downto 0) := (others => '0');
   signal coloumn : std_logic_vector(3 downto 0) := (others => '0');

 	--Outputs
   signal y : std_logic_vector(7 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 
  -- constant <clock>_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: Keypad_new PORT MAP (
          rows => rows,
          coloumn => coloumn,
          y => y
        );

   -- Clock process definitions
  -- <clock>_process :process
   --begin
		--<clock> <= '0';
		---wait for <clock>_period/2;
		--<clock> <= '1';
		--wait for <clock>_period/2;
   --end process;
 

   -- Stimulus process
   stim_proc: process
   begin	
   rows<="0001";
   coloumn<="0001";	
   wait for 100 ns;	
	
	rows<="0001";
   coloumn<="0010";	
   wait for 100 ns;
	
	rows<="0001";
   coloumn<="1000";	
   wait for 100 ns;
	
	
	
	rows<="0010";
   coloumn<="0001";	
   wait for 100 ns;
	
	rows<="0010";
   coloumn<="0010";	
   wait for 100 ns;
	
	rows<="0010";
   coloumn<="0100";	
   wait for 100 ns;
	
	rows<="0010";
   coloumn<="1000";	
   wait for 100 ns;
	
	
	rows<="0100";
   coloumn<="0001";	
   wait for 100 ns;
	
	rows<="0100";
   coloumn<="0010";	
   wait for 100 ns;
	
	rows<="0100";
   coloumn<="0100";	
   wait for 100 ns;
	
	rows<="0100";
   coloumn<="1000";	
   wait for 100 ns;
	
	rows<="1000";
   coloumn<="0001";	
   wait for 100 ns;
	
	rows<="1000";
   coloumn<="0010";	
   wait for 100 ns;
	
	rows<="1000";
   coloumn<="0100";	
   wait for 100 ns;
	
	rows<="1000";
   coloumn<="1000";	
   wait for 100 ns;
      --wait for <clock>_period*10;

      -- insert stimulus here 

      wait;
   end process;

END;

-------------------------------------------------------------------------------------------------------------------------------------------------------------------
#LCD Interfacing 
MAIN VHDL MODEL ( MVM )

library IEEE;
use IEEE.STD_LOGIC_1164.ALL; use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity LCD_FSM is
Port (		rst : in std_logic;	-- reset clk_12Mhz : in std_logic;	-- high freq. clock lcd_rs : out std_logic;	-- LCD RS control
lcd_en : out std_logic;	-- LCD Enable
lcd_data : out std_logic_vector(7 downto 0));	-- LCD Data port end LCD_FSM;

architecture Behavioral of LCD_FSM is

signal div : std_logic_vector(15 downto 0); --- delay timer 1 signal clk_fsm,lcd_rs_s: std_logic;
-- LCD controller FSM states
type state is (reset,func,mode,cur,clear,d0,d1,d2,d3,d4,hold); signal ps1,nx	: state;
signal dataout_s : std_logic_vector(7 downto 0); --- internal data command multiplexer begin
clk divider process(rst,clk_12Mhz) begin
if(rst = '1')then
div <= (others=>'0');
elsif( clk_12Mhz'event and clk_12Mhz ='1')then div <= div + 1;
 end if;

end process;

clk_fsm <= div(15);

----- Presetn state Register ----------------------- process(rst,clk_fsm)
begin
if(rst = '1')then
ps1	<= reset;
elsif (rising_edge(clk_fsm)) then ps1	<= nx;
end if;
end process;

----- state and output decoding process process(ps1)
begin case(ps1) is

 
when reset =>
 

nx	<= func; lcd_rs_s		<= '0';
dataout_s	<= "00111000";	-- 38h
 

when func	=>
nx	<= mode; lcd_rs_s		<= '0';
dataout_s	<= "00111000";	-- 38h

when mode	=>
nx	<= cur; lcd_rs_s		<= '0';
dataout_s	<= "00000110";	-- 06h
when cur	=>
nx	<= clear; lcd_rs_s		<= '0';
dataout_s	<= "00001100";	-- 0Ch curser at starting point of
 

 
when clear=>
 

nx	<= d0; lcd_rs_s		<= '0';
dataout_s	<= "00000001";	-- 01h
 

when d0	=>
lcd_rs_s	<= '1';
dataout_s	<= "01010000";	-- P ( Decimal = 80 , HEX = 50 )
nx	<= d1;

when d1	
=>
lcd_rs_s	

<= '1';	
	dataout_s
nx	<= d2;	<= "01001001";	-- I ( Decimal = 73 , HEX = 49 )
when d2	=>
lcd_rs_s	
<= '1';	
	dataout_s
nx	<= d3;	<= "01000011";	-- C ( Decimal = 67 , HEX = 43 )
when d3	=>
lcd_rs_s	
<= '1';	
	dataout_s
nx	<= d4;	<= "01010100";	-- T ( Decimal = 84 , HEX = 54 )

when d4	=>
lcd_rs_s	<= '1';
dataout_s	<= "00100000";	-- space ( Decimal = 32 , HEX = 20 ) nx	<= hold;

 



NULL
 
when hold	=>
lcd_rs_s	<= '0';
dataout_s	<= "00000000";	-- hold ( Decimal = 32 , HEX = 00 ) , nx	<= hold;
 
when others=>



end case; end process;

lcd_en <= clk_fsm; lcd_rs <= lcd_rs_s; lcd_data <= dataout_s;

end Behavioral;

#TESTBENCH VHDL MODEL ( TVM )


LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

ENTITY LCD_Test IS
END LCD_Test;

ARCHITECTURE behavior OF LCD_Test IS
-- Component Declaration for the Unit Under Test (UUT) COMPONENT LCD_FSM
PORT(
rst : IN std_logic; clk_12Mhz : IN std_logic; lcd_rs : OUT std_logic; lcd_en : OUT std_logic;
lcd_data : OUT std_logic_vector(7 downto 0)
);
END COMPONENT;

--Inputs
signal rst : std_logic := '0';
signal clk_12Mhz : std_logic := '0';

--Outputs
signal lcd_rs : std_logic; signal lcd_en : std_logic;
signal lcd_data : std_logic_vector(7 downto 0);
-- Clock period definitions
constant clk_12Mhz_period : time := 10 ns; BEGIN
-- Instantiate the Unit Under Test (UUT) uut: LCD_FSM PORT MAP (
rst => rst,
clk_12Mhz => clk_12Mhz, lcd_rs => lcd_rs,
lcd_en => lcd_en, lcd_data => lcd_data
);
-- Clock process definitions 
clk_12Mhz_process :process begin
clk_12Mhz <= '0';
wait for clk_12Mhz_period/2; clk_12Mhz <= '1';
wait for clk_12Mhz_period/2;
 

end process;
 



-- Stimulus process stim_proc: process begin
rst <= '1';
wait for 20 ns;

rst <= '0';
-- insert stimulus here

wait;
end process;

END;

-------------------------------------------------------------------------------------------------------------------------------------------------------------------
#MOD N -
#MAIN vhdl 
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity MOD_N_VHDL is
    Port ( clk : in  STD_LOGIC;
           rst : in  STD_LOGIC;
           count : out  INTEGER);
end MOD_N_VHDL;

architecture Behavioral of MOD_N_VHDL is
SIGNAL count_val : INTEGER := 0;

begin
process(clk,rst)
begin
if(rst = '1') then
	count_val <= 0;
elsif(clk'event and clk='1') then
	if(count_val = 30) then
		count_val <= 0;
	else
		count_val <= count_val+1;
	end if;
end if;
end process;
count <= count_val;

end Behavioral;

#testbench 

#LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_unsigned.all;
USE ieee.numeric_std.ALL;

ENTITY MOD_N_TB_vhd IS
END MOD_N_TB_vhd;

ARCHITECTURE behavior OF MOD_N_TB_vhd IS 

	-- Component Declaration for the Unit Under Test (UUT)
	COMPONENT MOD_N_VHDL
	PORT(
		clk : IN std_logic;
		rst : IN std_logic;          
		count : OUT INTEGER
		);
	END COMPONENT;

	--Inputs
	SIGNAL clk :  std_logic := '0';
	SIGNAL rst :  std_logic := '0';

	--Outputs
	SIGNAL count :  INTEGER;

BEGIN

	-- Instantiate the Unit Under Test (UUT)
	uut: MOD_N_VHDL PORT MAP(
		clk => clk,
		rst => rst,
		count => count
	);
	
	clk_process : PROCESS
	BEGIN 
	clk <= '0';
	wait for 10 ns;
	clk <= '1';
	wait for 10 ns;
	end process;

	tb : PROCESS
	BEGIN

		-- Wait 100 ns for global reset to finish
		wait for 100 ns;

		-- Place stimulus here
		rst <= '1';
		wait for 100 ns;
		rst <= '0';
		

		wait; -- will wait forever
	END PROCESS;

END;

-----------------------------------------------------------------------------------------------------------------------------------------------------------------
#Javascript exp 9 calculator 
PROGRAM:
<!DOCTYPE html>
<html>
<head>
  <title>Simple JavaScript Calculator</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background: linear-gradient(to right, #fbc2eb, #a6c1ee);
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
    }
    .calculator {
      background: #fff;
      padding: 20px;
      border-radius: 15px;
      box-shadow: 0px 10px 20px rgba(0,0,0,0.2);
      width: 300px;
      text-align: center;
    }
    input[type="text"] {
      width: 90%;
      padding: 10px;
      font-size: 18px;
      margin-bottom: 10px;
      border-radius: 8px;
      border: 1px solid #ccc;
    }
    button {
      width: 60px;
      height: 50px;
      margin: 5px;
      font-size: 18px;
      border-radius: 8px;
      border: none;
      cursor: pointer;
      transition: 0.3s;
    }
    button:hover {
      opacity: 0.8;
    }
    .result-tab {
      margin-top: 15px;
      background: #f0f0f0;
      padding: 10px;
      border-radius: 8px;
      font-size: 20px;
      min-height: 40px;
    }
    .operator { background-color: #ffb347; }
    .number { background-color: #6a82fb; color: white; }
    .equal { background-color: #f78ca0; color: white; width: 130px; }
    .clear { background-color: #f4f4f4; width: 130px; }
  </style>
</head>
<body>

<div class="calculator">
  <h2>JS Calculator</h2>
  <input type="text" id="input" placeholder="Enter number(s)">
  
  <div>
    <!-- Number Buttons -->
    <button class="number" onclick="appendNumber('1')">1</button>
    <button class="number" onclick="appendNumber('2')">2</button>
    <button class="number" onclick="appendNumber('3')">3</button>
    <button class="operator" onclick="appendOperator('+')">+</button>
  </div>
  <div>
    <button class="number" onclick="appendNumber('4')">4</button>
    <button class="number" onclick="appendNumber('5')">5</button>
    <button class="number" onclick="appendNumber('6')">6</button>
    <button class="operator" onclick="appendOperator('-')">-</button>
  </div>
  <div>
    <button class="number" onclick="appendNumber('7')">7</button>
    <button class="number" onclick="appendNumber('8')">8</button>
    <button class="number" onclick="appendNumber('9')">9</button>
    <button class="operator" onclick="appendOperator('*')">×</button>
  </div>
  <div>
    <button class="number" onclick="appendNumber('0')">0</button>
    <button class="operator" onclick="appendOperator('.')">.</button>
    <button class="operator" onclick="squareNumber()">x²</button>
    <button class="operator" onclick="appendOperator('/')">÷</button>
  </div>
  <div>
    <button class="equal" onclick="calculate()">=</button>
    <button class="clear" onclick="clearInput()">C</button>
  </div>

  <div class="result-tab" id="result">Result: </div>
</div>

<script>
  let input = document.getElementById('input');
  let resultTab = document.getElementById('result');

  function appendNumber(num) {
    input.value += num;
  }

  function appendOperator(op) {
    if(input.value === '') {
      alert('Enter a number first!');
      return;
    }
    input.value += op;
  }

  function squareNumber() {
    if(input.value === '' || isNaN(input.value)) {
      alert('Enter a valid number to square!');
      return;
    }
    input.value = Math.pow(Number(input.value), 2);
  }

  function calculate() {
    if(input.value === '') {
      alert('Enter a valid expression!');
      return;
    }
    try {
      // Evaluate the expression
      let result = eval(input.value);
      if(isNaN(result)) {
        alert('Invalid calculation!');
        resultTab.innerHTML = 'Result: Error';
      } else {
        resultTab.innerHTML = 'Result: ' + result;
      }
    } catch {
      alert('Invalid expression!');
      resultTab.innerHTML = 'Result: Error';
    }
  }

  function clearInput() {
    input.value = '';
    resultTab.innerHTML = 'Result: ';
  }
</script>

</body>
</html>







