{
  description = "RTL Lab — open hardware development shell";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    nixchip = {
      url = "github:helcel-net/nixchip";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, nixchip }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ nixchip.overlays.default ];
      };
      hw = pkgs.nixchip;

      # nixchip packages used in this shell — passed as an attrset so
      # mkNixchipVarsHook can derive PKGNAME_{HOME,BIN,LIB,INCLUDE} for each.
      hwPkgs = {
        inherit (hw) verilator systemc verible;
      };
    in
    {
      devShells.${system}.default = pkgs.mkShellNoCC {
        packages = builtins.attrValues hwPkgs ++ [
          pkgs.gcc
          pkgs.clang-tools
          pkgs.yosys
        ];

        shellHook = nixchip.lib.mkNixchipVarsHook hwPkgs + ''
          export RTL_LAB_HOME="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
          export SYSTEMC_LIBDIR="$SYSTEMC_LIB"
        '';
      };
    };
}
