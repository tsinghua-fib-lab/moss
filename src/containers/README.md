# simulet::Vector

目的：提供一种从GPU端收集数据到CPU端的容器。

示例代码
```cpp
__global__ void compute(simulet::DVector<int>* a) {
  auto id = threadIdx.x + blockIdx.x * blockDim.x;
  // 向容器中添加数据
  if (id % 2 == 1) {
    a->Append(id);
  }
}

int main() {
  // 创建容器用于接收数据
  simulet::Vector<int> a;
  // 用.Cuda获取将容器的GPU部分，传给kernel
  compute<<<2, 4>>>(a.Cuda());
  // 同步
  cudaDeviceSynchronize();
  // 用.Cpu将数据读到CPU上
  for (auto& i : a.Cpu()) {
    printf("%d ", i);
  }
  printf("\n");
  CUCHECK(cudaGetLastError());
}
```

# simulet::Post

目的：实现GPU发起请求->CPU处理请求->GPU接收回复的流程

`Post<U,V>`
* GPU端
  * .Send(u) -> id：发送消息
  * .Receive(id) -> V*(&v|nullptr)：接收回复
* CPU端
  * .Cuda() -> DPost*：获取用于传给kernel的对象
  * .Cpu() -> {(id,u), ...}：获取所有消息
  * .Reply(id,v)：回复消息
  * .WaitFree()：等待有资源可用
  * .WaitAll()：等待所有消息都被回复

上述“发送-处理-接收”为一轮操作，可以设置允许在一轮中的未全部回复的情况下进入下一轮，但由于这样会导致资源不释放，因此设置一个上限