let
     pkgs = import <nixpkgs> {};
     stdenv = pkgs.stdenv;
in rec {
          cudaEnv = stdenv.mkDerivation rec {
          name = "cuda-env";
          version = "1.1.1.1";
          src = ./.;
          buildInputs = [ pkgs.opencv3 pkgs.pkgs.eigen pkgs.cgal pkgs.boost pkgs.gmp pkgs.mpfr pkgs.glog pkgs.google-gflags pkgs.ceres-solver pkgs.cmake pkgs.gnumake pkgs.pkgconfig pkgs.stdenv pkgs.cudatoolkit ];
     };
}
