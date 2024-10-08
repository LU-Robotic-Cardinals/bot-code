{pkgs ? import <nixpkgs> {}}:
pkgs.mkShell {
  #nativeBuildInputs = with pkgs; [];  # only needed for developing nix/nixos
  buildInputs = with pkgs; [
    # common build inputs
    direnv
    xclip
    duf
    eza
    fd
    clang
    cmake
    gcc
    # project-specific build inputs
    #bun
    #deno
    #rustup
    #nodejs
    #nodePackages.pnpm
  ];
  env = {
    #DENO_BIN="${pkgs.deno}/bin/deno";
    #NODE_BIN="${pkgs.nodejs}/bin/nodejs";
  };
  shellHook = ''
    zsh
  '';
}