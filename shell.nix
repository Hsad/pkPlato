{ pkgs ? import <nixos> {} }:
let
  # Import the unstable channel
  unstable = import (fetchTarball "https://github.com/NixOS/nixpkgs/archive/nixos-unstable.tar.gz") {};
in
pkgs.mkShell
{
  nativeBuildInputs = with pkgs; [
    gcc
    unstable.odin
    raylib
    glfw
    xorg.libX11
    xorg.libXrandr
    xorg.libXinerama
    xorg.libXcursor
    xorg.libXi
    xorg.xeyes
    mesa
    libglvnd
    git
    bash
    watchexec
    emscripten
  ];

  shellHook = ''
    export LD_LIBRARY_PATH=${pkgs.xorg.libX11}/lib:${pkgs.xorg.libXrandr}/lib:${pkgs.xorg.libXinerama}/lib:${pkgs.xorg.libXcursor}/lib:${pkgs.xorg.libXi}/lib:${pkgs.raylib}/lib:${pkgs.mesa}/lib:${pkgs.libglvnd}/lib:$LD_LIBRARY_PATH
    export LIBGL_ALWAYS_SOFTWARE=1
    export DISPLAY=:0
    export XDG_SESSION_TYPE=x11
    export GDK_BACKEND=wayland
    export SDL_VIDEODRIVER=wayland
    echo "Odin environment running"
    echo "watchexec -w source/game.odin bash build_hot_reload.sh"
  '';
}
