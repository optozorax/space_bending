# Space Bending

This is the code behind https://optozorax.github.io/space_bending/


## How to make changes and run it locally

1. install Rust using https://rustup.rs/ 
1. `cd wasm`
1. `cargo install wasm-pack` install the wasm builder
1. `cargo install miniserve` install a webserver to serve local files
1. `wasm-pack build --target web`  build rust code into a wasm.js file
1. `miniserve ..` host a webserver that has index.html at the root
1. Open http://localhost:8080/index.html in your browser

When you make code changes, you can leave miniserve running, in another terminal tab
re-run only `wasm-pack build --target web`.
Then refresh your browser to see the new version.


## Code architecture

- The UI and render loop is defined in JavaScript as part of index.html
- The Mesh creation and simulation is implemented in rust in the wasm directory.
