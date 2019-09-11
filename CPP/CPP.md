# CPP知识点总结

## new/delete和malloc/free
- a. malloc是标准库函数，而new是C++的运算符(类似于加减乘除这样的)，运算符是可以被编译器优化重载的，但是库函数不能；
- b. new返回对象的指针，而malloc返回的永远为void\*;
- c. new的对象在**自由存储区**，而malloc的内存在堆上；
- d. new的时候，操作会调用析构函数，而malloc的时候不会；
- e. new失败的时候，会抛出异常，而malloc直接返回NULL；