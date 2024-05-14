# 开发共识

* 命名规则
  * 函数名、类名、结构体名用首字母大写的格式，例如 ThisIsAFunction
  * 命名空间、变量名用小写下划线，例如 this_is_a_variable
  * 如果一个host函数在内部使用了异步的cuda操作并且没有做同步(cudaDeviceSynchronize)，函数名在尾部加Async，例如PrepareAsync
  * 当多个对象实现语义上相同的函数时，使用相同的函数名称并用命名空间加以区分，aoi::Init和lane::Init
  * 使用proto中的数据类型时，在protos.h文件中用using定义别名，以Pb开头，但枚举类型不加Pb
* 其他
  * 头文件中的内容应该只包括需要在其他地方使用的部分，不要包括多余的东西
  * 大部分情况下数据都使用32位类型，特别是在GPU上，用float/uint，后者在types.h中定义
  * 当需要定义一个共享的全局变量时
    * 在.cc/.cu文件中写定义，如int a
    * 在.h/.cuh中写声明，如extern int a
    * 想要使用变量a只需要在.cc/.cu文件中include对应的.h/.cuh文件

# 注意事项

* 模拟器支持Save/Load功能以保存和恢复运行状态，因此后续的修改必须检查是否需要修改Save/Load
* 人的初始化需要设置p.runtime.lane
