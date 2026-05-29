// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   SystemC demo harness for the `make sim-sc` flow. Demonstrates a Verilated
//   SystemVerilog DUT and a native SystemC module simulating together under the
//   SystemC kernel, bundled inside the top_mul_s design top:
//
//     sc_main
//       |- sc_clock  clk
//       |- driver    (SC testbench: stimulus + checker)
//       |- top_mul_s (SystemC design top)
//            |- Vmul_s dut   (Verilated SV combinational signed multiplier)
//            |- acc    accu  (native SystemC clocked accumulator)
//
//   The driver feeds random signed operands to the top; inside it the
//   multiplier's product feeds the SystemC accumulator; the driver checks the
//   accumulator's running sum against an independently computed reference and
//   reports PASS/FAIL, then stops the simulation.
//
//   Widths below must match the mul_s elaboration parameters. They are fixed to
//   the DUT defaults (8x8); overriding IN_WIDTH_A/IN_WIDTH_B via PARAMS requires
//   updating A_W/B_W here, since SystemC port widths are compile-time.
// -----------------------------------------------------------------------------

#include <systemc.h>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "verilated.h"
#include "verilated_vcd_sc.h"

#include "top_mul_s.hpp"

#ifndef CLK_PERIOD_NS
#define CLK_PERIOD_NS 1.0
#endif
#ifndef IN_WIDTH_A
#define IN_WIDTH_A 8
#endif
#ifndef IN_WIDTH_B
#define IN_WIDTH_B 8
#endif
#ifndef N_TESTS
#define N_TESTS 1000
#endif
#ifndef SEED
#define SEED 1
#endif

static const int A_W   = IN_WIDTH_A;
static const int B_W   = IN_WIDTH_B;
static const int OUT_W = A_W + B_W;

static int rand_signed(int w) {
    const int range = 1 << w;
    return (std::rand() % range) - (range >> 1);
}

static uint32_t to_bits(int v, int w) {
    return static_cast<uint32_t>(v) & ((1u << w) - 1u);
}

SC_MODULE(driver) {
    sc_in<bool>      clk_i;
    sc_out<bool>     rst_o;
    sc_out<uint32_t> a_o;
    sc_out<uint32_t> b_o;
    sc_in<int>       sum_i;

    void run() {
        std::srand(SEED);

        rst_o.write(true);
        a_o.write(0);
        b_o.write(0);
        wait();
        rst_o.write(false);

        long expected = 0;
        for (int i = 0; i < N_TESTS; i++) {
            const int a = rand_signed(A_W);
            const int b = rand_signed(B_W);
            a_o.write(to_bits(a, A_W));
            b_o.write(to_bits(b, B_W));
            expected += static_cast<long>(a) * b;
            wait();
        }

        a_o.write(0);
        b_o.write(0);
        wait();
        wait();

        const long got = sum_i.read();
        if (got == expected) {
            std::cout << "[tb_top_mul_s] PASS: " << N_TESTS
                      << " products accumulated, sum = " << got << std::endl;
        } else {
            std::cout << "[tb_top_mul_s] FAIL: sum = " << got
                      << ", expected = " << expected << std::endl;
        }
        sc_stop();
    }

    SC_CTOR(driver) {
        SC_THREAD(run);
        sensitive << clk_i.pos();
    }
};

int sc_main(int argc, char* argv[]) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    sc_clock            clk("clk", CLK_PERIOD_NS, SC_NS);
    sc_signal<bool>     rst;
    sc_signal<uint32_t> a;
    sc_signal<uint32_t> b;
    sc_signal<int>      sum;

    driver drv("drv");
    drv.clk_i(clk);
    drv.rst_o(rst);
    drv.a_o  (a);
    drv.b_o  (b);
    drv.sum_i(sum);

    top_mul_s<OUT_W> top("top");
    top.clk_i(clk);
    top.rst_i(rst);
    top.a_i  (a);
    top.b_i  (b);
    top.sum_o(sum);

    sc_start(SC_ZERO_TIME);

    VerilatedVcdSc* tfp = new VerilatedVcdSc;
    top.dut.trace(tfp, 99);
    tfp->open("activity.vcd");

    sc_start();

    tfp->close();
    delete tfp;
    return 0;
}
