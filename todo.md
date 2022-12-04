# TODO

- The default hit/miss and min/max for ohmpop are not tuned for NDT (based on VLP-16 input)
  - Max is certainly too low and will result in erosion
- (More) GPU optimisation pass. Since memory access is so irregular, best to look at avoiding branching
  - Look for effects of different GPU hardware
  - Look for differences between OpenCL/CUDA
- Convert to RAII (new/delete have been targetted)
- `inline` keyword is overused for methods inlined in the class body. This is a legacy of when MSC would not imply inline. It's hard to say when this changed, anecdotally speaking, but the documentation from [Visual Studio 2015](https://learn.microsoft.com/en-us/cpp/cpp/inline-functions-cpp?view=msvc-140) is clear that's its no longer necessary, or just a misinterpretation
