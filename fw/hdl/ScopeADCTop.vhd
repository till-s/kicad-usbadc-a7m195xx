library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;

library unisim;
use     unisim.vcomponents.all;

use     work.UlpiPkg.all;
use     work.Usb2Pkg.all;
use     work.Usb2AppCfgPkg.all;
use     work.CommandMuxPkg.all;

entity ScopeADCTop is
   generic (
      SPEED_GRADE_G            : natural range 1 to 3 := 1
   );
   port (
      ulpiClk                  : inout std_logic;
      ulpiRstb                 : out   std_logic := '1';
      ulpiDir                  : in    std_logic;
      ulpiNxt                  : in    std_logic;
      ulpiStp                  : inout std_logic;
      ulpiDat                  : inout std_logic_vector(7 downto 0);

      led                      : out   std_logic_vector(12 downto 0) := (others => '0');

      i2cScl                   : inout std_logic;
      i2cSda                   : inout std_logic;

      gpioDat                  : inout std_logic := 'Z';
      gpioDir                  : inout std_logic := 'Z';

      adcSClk                  : out   std_logic := '0';
      adcSDIO                  : inout std_logic := '0';
      adcCSb                   : out   std_logic := '1';

      adcDClk                  : in    std_logic;
      adcDatDDR                : in    std_logic_vector(9 downto 0);
      adcDORDDR                : in    std_logic;
      adcSync                  : out   std_logic := '0';

      pgaSClk                  : out   std_logic := '0';
      pgaSDat                  : out   std_logic := '0';
      pgaCSb                   : out   std_logic_vector(1 downto 0) := (others => '1');

      pllClk                   : in    std_logic
   );

   attribute PULLTYPE          : string;

   attribute PULLTYPE          of ulpiClk : signal is "PULLDOWN";

end entity ScopeADCTop;

architecture rtl of ScopeADCTop is

   -- for speed grade 1; 120MHz works with IDELAY_TAPS => 18
   constant ADC_FREQ_C         : real    := 120.0E6;
   constant IDELAY_TAPS_C      : natural := 18;

   -- speed grade 2; 130MHz works with IDELAY_TAPS = 16
--   constant ADC_FREQ_C         : real    := 130.0E6;
--   constant IDELAY_TAPS_C      : natural := 16;

   constant LD_FIFO_OUT_C      : natural := 11;
   constant LD_FIFO_INP_C      : natural := 11;

   constant BB_SPI_CSb_C       : natural := 0;
   constant BB_SPI_SCK_C       : natural := 1;
   constant BB_SPI_MSO_C       : natural := 2;
   constant BB_SPI_MSI_C       : natural := 3;

   constant BB_I2C_SDA_C       : natural := 4;
   constant BB_I2C_SCL_C       : natural := 5;

   constant BB_SPI_T_C         : natural := 6;
   constant BB_XXX_XXX_C       : natural := 7;

   constant BB_INIT_C          : std_logic_vector(7 downto 0) := x"F1";

   constant ULPI_CLK_FREQ_C    : real    := 60.0E6;
   constant ACM_CLK_FREQ_C     : real    := ULPI_CLK_FREQ_C;
   constant DLY_REF_CLK_FREQ_C : real    := 195.0E6;

   constant ADC_BITS_C         : natural := 8;
   constant MEM_DEPTH_C        : natural := 64*1024;

   signal ulpiIb               : UlpiIbType := ULPI_IB_INIT_C;
   signal ulpiOb               : UlpiObType := ULPI_OB_INIT_C;

   signal ulpiClkDly           : std_logic;

   signal ulpiClk_i            : std_logic;
   signal ulpiClk_o            : std_logic := '0';
   signal ulpiClk_t            : std_logic := '1';

   signal usb2Clk              : std_logic;
   signal acmFifoClk           : std_logic;
   signal acmFifoRst           : std_logic    := '0';

   signal acmFifoOutDat        : Usb2ByteType;
   signal acmFifoOutEmpty      : std_logic;
   signal acmFifoOutRen        : std_logic    := '1';
   signal acmFifoInpDat        : Usb2ByteType := (others => '0');
   signal acmFifoInpFull       : std_logic;
   signal acmFifoInpWen        : std_logic    := '0';

   signal acmFifoInpMinFill    : unsigned(LD_FIFO_INP_C - 1 downto 0) := (others=> '0');
   signal acmFifoInpTimer      : unsigned(32 - 1 downto 0) := (others=> '0');

   signal acmFifoLocal         : std_logic    := '1';

   signal usbMMCMLocked        : std_logic;

   signal subCmdBB             : SubCommandBBType;

   signal fifoRDat             : Usb2ByteType;
   signal fifoRRdy             : std_logic;
   signal fifoRVld             : std_logic;
   signal fifoWDat             : Usb2ByteType;
   signal fifoWRdy             : std_logic;
   signal fifoWVld             : std_logic;

   -- extra bit is DOR / overflow
   signal adcDDRLoc            : std_logic_vector(ADC_BITS_C downto 0) := (others => '0');
   signal adcDcmLocked         : std_logic    := '0';
   
   signal bbi                  : std_logic_vector(7 downto 0) := (others => '1');
   signal bbo                  : std_logic_vector(7 downto 0) := (others => '1');

   signal cnt                  : integer      := -1;
   signal blnk                 : std_logic    := '0';

   signal pllClkBuf            : std_logic;

   signal ulpiRstTimer         : signed(16 downto 0) := '0' & x"ffff";

   signal usb2RstTimer         : signed(8 downto 0) :=  (others => '1');

   signal usb2Rst              : std_logic := '0';
   signal ulpiRst              : std_logic := '0';
   signal ulpiForceStp         : std_logic := '0';

   signal stp_i                : std_logic;
   signal stp_t                : std_logic := '0';
   
   signal dlyRefClk            : std_logic;

   component ila_0 is
      port (
         clk             : in  std_logic;
         probe0          : in  std_logic_vector(63 downto 0) := (others => '0');
         probe1          : in  std_logic_vector(63 downto 0) := (others => '0');
         probe2          : in  std_logic_vector(63 downto 0) := (others => '0');
         probe3          : in  std_logic_vector(63 downto 0) := (others => '0');
         trig_in         : in  std_logic := '0';
         trig_in_ack     : out std_logic;
         trig_out        : out std_logic;
         trig_out_ack    : in  std_logic := '1'
      );
   end component ila_0;

   signal trg                  : std_logic := '1';
   signal trgack               : std_logic := '0';

begin

   U_IOBUF_ULPI_CLK : IOBUF
      port map (
         IO      => ulpiClk,
         I       => ulpiClk_o,
         O       => ulpiClk_i,
         T       => ulpiClk_t
      );

   usb2Clk    <= ulpiClkDly;
   acmFifoClk <= ulpiClkDly;

   U_BUF_PLLCLK : BUFG port map ( I => pllClk,  O => pllClkBuf );

   P_BLINK : process ( ulpiClkDly ) is
   begin
      if ( rising_edge( ulpiClkDly ) ) then
         if ( cnt < 0 ) then
            blnk <= not blnk;
            cnt  <= 30000000;
         else
            cnt  <= cnt - 1;
         end if;
         if ( usb2RstTimer( usb2RstTimer'left ) = '1' ) then
            usb2RstTimer <= usb2RstTimer - 1;
         end if;
      end if;
   end process P_BLINK;

   ulpiClk_o     <= '0';
   ulpiClk_t     <= ulpiRstTimer( ulpiRstTimer'left );
   ulpiRstb      <= ulpiRstTimer( ulpiRstTimer'left );

   usb2Rst       <= usb2RstTimer( usb2RstTimer'left );
   ulpiRst       <= usb2RstTimer( usb2RstTimer'left );
   stp_t         <= usb2RstTimer( usb2RstTimer'left );

   P_ULPI_RST : process ( pllClkBuf ) is
   begin
      if ( rising_edge( pllClkBuf ) ) then
         if ( ulpiRstTimer >= 0 and trgack = '1' ) then
            ulpiRstTimer <= ulpiRstTimer - 1;
         end if;
         if ( trg = '1' ) then
            trgack <= '1';
         end if;
      end if;
   end process P_ULPI_RST;

   led(led'left downto 1) <= (others => '0');
   led(0)                 <= blnk;

   G_ILAS : if ( false ) generate

   U_ILA_RST : ila_0
      port map (
         clk                         => pllClkBuf,
         probe0(ulpiRstTimer'range)  => std_logic_vector(ulpiRstTimer),
         probe0(ulpiRstTimer'length) => ulpiClk_i,
         probe0(ulpiRstTimer'length + 1) => ulpiIb.dir,
         probe0(ulpiRstTimer'length + 2) => stp_i,
         probe0(63 downto ulpiRstTimer'length + 3) => (others => '0'),

         trig_out                    => trg,
         trig_out_ack                => trgack
      );

   U_ILA_FIFO : ila_0
      port map (
         clk                         => ulpiClkDly,
         probe0(7 downto 0)          => acmFifoOutDat,
         probe0(8         )          => acmFifoOutEmpty,
         probe0(9         )          => acmFifoOutRen,
         probe0(15 downto 10)        => (others => '0'),
         probe0(23 downto 16)        => acmFifoInpDat,
         probe0(24        )          => acmFifoInpFull,
         probe0(25        )          => acmFifoInpWen,
         probe0(63 downto 26)        => (others => '0')
      );

   end generate G_ILAS;


   U_USB_DEV : entity work.Usb2ExampleDev
      generic map (
         ULPI_CLK_MODE_INP_G       => false,
         DESCRIPTORS_G             => USB2_APP_DESCRIPTORS_C,
         DESCRIPTORS_BRAM_G        => false,
         LD_ACM_FIFO_DEPTH_INP_G   => LD_FIFO_INP_C,
         LD_ACM_FIFO_DEPTH_OUT_G   => LD_FIFO_OUT_C,
         CDC_ACM_ASYNC_G           => false,
         MARK_DEBUG_ULPI_IO_G      => false,
         MARK_DEBUG_PKT_TX_G       => false,
         MARK_DEBUG_PKT_RX_G       => false,
         MARK_DEBUG_PKT_PROC_G     => false
      )
      port map (
         usb2Clk                   => usb2Clk,
         usb2Rst                   => usb2Rst,
         ulpiRst                   => ulpiRst,
         ulpiIb                    => ulpiIb,
         ulpiOb                    => ulpiOb,
         ulpiForceStp              => ulpiForceStp,

         usb2HiSpeedEn             => '1',

         acmFifoClk                => acmFifoClk,
         acmFifoOutDat             => acmFifoOutDat,
         acmFifoOutEmpty           => acmFifoOutEmpty,
         acmFifoOutRen             => acmFifoOutRen,
         acmFifoInpDat             => acmFifoInpDat,
         acmFifoInpFull            => acmFifoInpFull,
         acmFifoInpWen             => acmFifoInpWen,

         acmFifoInpMinFill         => acmFifoInpMinFill,
         acmFifoInpTimer           => acmFifoInpTimer,
         acmFifoLocal              => acmFifoLocal
      );

   B_BUFS : block is
   begin

      ulpiIb.dir        <= ulpiDir;
      ulpiIb.nxt        <= ulpiNxt;

      stp_i             <= ulpiStp;
      ulpiStp           <= 'Z' when stp_t = '1' else ulpiOb.stp;

      ulpiIb.dat        <= ulpiDat;
      ulpiDat           <= (others => 'Z') when ulpiIb.dir = '1' else ulpiOb.dat;

      bbi(BB_I2C_SDA_C) <= i2cSda;
      i2cSda            <= 'Z' when bbo(BB_I2C_SDA_C) = '1' else '0';
      bbi(BB_I2C_SCL_C) <= i2cScl;
      i2cScl            <= 'Z' when bbo(BB_I2C_SCL_C) = '1' else '0';

      P_CS_MUX : process ( bbo, subCmdBB, adcSDIO ) is
      begin
         adcCSb  <= '1';
         pgaCSb  <= (others => '1');
         adcSDIO <= 'Z';

         pgaSClk <= bbo(BB_SPI_SCK_C);
         adcSClk <= bbo(BB_SPI_SCK_C);

         pgaSDat <= bbo(BB_SPI_MSO_C);

         bbi(BB_SPI_MSI_C) <= '0';

         if    ( subCmdBB = CMD_BB_SPI_ADC_C ) then
            adcCSb      <= bbo(BB_SPI_CSb_C);
            if ( bbo(BB_SPI_T_C) = '0' ) then
               adcSDIO  <= bbo(BB_SPI_MSO_C);
            end if;
            bbi(BB_SPI_MSI_C) <= adcSDIO;
         elsif ( subCmdBB = CMD_BB_SPI_VGA_C ) then
            pgaCSb(0)         <= bbo(BB_SPI_CSb_C);
         elsif ( subCmdBB = CMD_BB_SPI_VGB_C ) then
            pgaCSb(1)         <= bbo(BB_SPI_CSb_C);
         end if;
      end process P_CS_MUX;

   end block B_BUFS;

   B_MMCM : block is

      constant CLK_MULT_F_C    : real    :=  13.000;
      constant REF_PERIOD_C    : real    :=  16.666;
      constant CLK0_DIV_C      : natural :=  13;
      constant CLK2_DIV_C      : natural :=   4;
      constant CLK3_DIV_C      : natural :=  13;
      constant REF_CLK_DIV_C   : natural :=  1;
      -- phase must be a multiple of 45/CLK0_DIV_G; ideally 15deg (proven timing)
      constant CLKOUT0_PHASE_C : real    :=  13.846;
      constant CLKOUT1_PHASE_C : real    :=  0.0;

      signal clkFbI, clkFbO    : std_logic;
      signal clk0Nb            : std_logic;
      signal clk2Nb            : std_logic;

   begin

      U_BUFG_FBCK   : BUFG port map ( I => clkFbO, O => clkFbI     );
      U_BUFG_CLK0   : BUFG port map ( I => clk0Nb, O => ulpiClkDly );

      U_MMCM : MMCME2_BASE
         generic map (
            BANDWIDTH => "OPTIMIZED",  -- Jitter programming (OPTIMIZED, HIGH, LOW)
            CLKFBOUT_MULT_F => CLK_MULT_F_C,    -- Multiply value for all CLKOUT (2.000-64.000).
            CLKFBOUT_PHASE => 0.0,     -- Phase offset in degrees of CLKFB (-360.000-360.000).
            CLKIN1_PERIOD => REF_PERIOD_C,      -- Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
            -- CLKOUT0_DIVIDE - CLKOUT6_DIVIDE: Divide amount for each CLKOUT (1-128)
            CLKOUT1_DIVIDE => CLK0_DIV_C,
            CLKOUT2_DIVIDE => CLK2_DIV_C,
            CLKOUT3_DIVIDE => CLK3_DIV_C,
            CLKOUT4_DIVIDE => CLK0_DIV_C,
            CLKOUT5_DIVIDE => CLK0_DIV_C,
            CLKOUT6_DIVIDE => CLK0_DIV_C,
            CLKOUT0_DIVIDE_F => real(CLK0_DIV_C),   -- Divide amount for CLKOUT0 (1.000-128.000).
            -- CLKOUT0_DUTY_CYCLE - CLKOUT6_DUTY_CYCLE: Duty cycle for each CLKOUT (0.01-0.99).
            CLKOUT0_DUTY_CYCLE => 0.5,
            CLKOUT1_DUTY_CYCLE => 0.5,
            CLKOUT2_DUTY_CYCLE => 0.5,
            CLKOUT3_DUTY_CYCLE => 0.5,
            CLKOUT4_DUTY_CYCLE => 0.5,
            CLKOUT5_DUTY_CYCLE => 0.5,
            CLKOUT6_DUTY_CYCLE => 0.5,
            -- CLKOUT0_PHASE - CLKOUT6_PHASE: Phase offset for each CLKOUT (-360.000-360.000).
            CLKOUT0_PHASE => CLKOUT0_PHASE_C,
            CLKOUT1_PHASE => CLKOUT1_PHASE_C,
            CLKOUT2_PHASE => 0.0,
            CLKOUT3_PHASE => 0.0,
            CLKOUT4_PHASE => 0.0,
            CLKOUT5_PHASE => 0.0,
            CLKOUT6_PHASE => 0.0,
            CLKOUT4_CASCADE => FALSE,  -- Cascade CLKOUT4 counter with CLKOUT6 (FALSE, TRUE)
            DIVCLK_DIVIDE => REF_CLK_DIV_C, -- Master division value (1-106)
            REF_JITTER1 => 0.0,        -- Reference input jitter in UI (0.000-0.999).
            STARTUP_WAIT => FALSE      -- Delays DONE until MMCM is locked (FALSE, TRUE)
         )
         port map (
            -- Clock Outputs: 1-bit (each) output: User configurable clock outputs
            CLKOUT0 => clk0Nb,     -- 1-bit output: CLKOUT0
            CLKOUT0B => open,   -- 1-bit output: Inverted CLKOUT0
            CLKOUT1 => open,     -- 1-bit output: CLKOUT1
            CLKOUT1B => open,   -- 1-bit output: Inverted CLKOUT1
            CLKOUT2 => clk2Nb,     -- 1-bit output: CLKOUT2
            CLKOUT2B => open,   -- 1-bit output: Inverted CLKOUT2
            CLKOUT3 => open,     -- 1-bit output: CLKOUT3
            CLKOUT3B => open,   -- 1-bit output: Inverted CLKOUT3
            CLKOUT4 => open,     -- 1-bit output: CLKOUT4
            CLKOUT5 => open,     -- 1-bit output: CLKOUT5
            CLKOUT6 => open,     -- 1-bit output: CLKOUT6
            -- Feedback Clocks: 1-bit (each) output: Clock feedback ports
            CLKFBOUT => clkFbO,   -- 1-bit output: Feedback clock
            CLKFBOUTB => open, -- 1-bit output: Inverted CLKFBOUT
            -- Status Ports: 1-bit (each) output: MMCM status ports
            LOCKED => usbMMCMLocked, -- 1-bit output: LOCK
            -- Clock Inputs: 1-bit (each) input: Clock input
            CLKIN1 => ulpiClk_i,       -- 1-bit input: Clock
            -- Control Ports: 1-bit (each) input: MMCM control ports
            PWRDWN => '0',       -- 1-bit input: Power-down
            RST => '0',             -- 1-bit input: Reset
            -- Feedback Clocks: 1-bit (each) input: Clock feedback ports
            CLKFBIN => clkFbI      -- 1-bit input: Feedback clock
         );

      U_DLY_REF_BUF : BUFG
         port map (
            I => clk2Nb,
            O => dlyRefClk
         );
   end block B_MMCM;

   fifoRDat      <= acmFifoOutDat;
   fifoRVld      <= not acmFifoOutEmpty;
   acmFifoOutRen <= fifoRRdy;

   acmFifoInpDat <= fifoWDat;
   acmFifoInpWen <= fifoWVld;
   fifoWRdy      <= not acmFifoInpFull;

   U_CMD_WRAPPER : entity work.CommandWrapper
      generic map (
         I2C_SCL_G                => BB_I2C_SCL_C,
         BBO_INIT_G               => BB_INIT_C,
         FIFO_FREQ_G              => ACM_CLK_FREQ_C,
         ADC_FREQ_G               => ADC_FREQ_C,
         ADC_BITS_G               => ADC_BITS_C,
         MEM_DEPTH_G              => MEM_DEPTH_C,
         DDR_TYPE_G               => "IDDR",
         DLY_REF_MHZ_G            => (DLY_REF_CLK_FREQ_C/1.0E6),
         IDELAY_TAPS_G            => IDELAY_TAPS_C
      )
      port map (
         clk                      => acmFifoClk,
         rst                      => acmFifoRst,

         datIb                    => fifoRDat,
         vldIb                    => fifoRVld,
         rdyIb                    => fifoRRdy,
         datOb                    => fifoWDat,
         vldOb                    => fifoWVld,
         rdyOb                    => fifoWRdy,

         bbo                      => bbo,
         bbi                      => bbi,
         subCmdBB                 => subCmdBB,

         adcClk                   => adcDClk,
         adcDataDDR               => adcDDRLoc,
         smplClk                  => open,
         adcDcmLocked             => adcDcmLocked,

         dlyRefClk                => dlyRefClk
      );

   adcDDRLoc <= (adcDatDDR(adcDatDDR'left downto adcDatDDR'left - ADC_BITS_C + 1) & adcDORDDR);

end architecture rtl;
