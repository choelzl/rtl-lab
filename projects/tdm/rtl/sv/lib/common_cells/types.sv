
`ifndef LIB_TYPES_SV
`define LIB_TYPES_SV

typedef enum logic {
    NtoM,
    onetoM
} bus_type_e;

package addr_map_rule_pkg;
  typedef struct packed {
    logic [31:0] idx;
    logic [31:0] start_addr;
    logic [31:0] end_addr;
  } addr_map_rule_t;
endpackage

`endif // LIB_TYPES_SV
