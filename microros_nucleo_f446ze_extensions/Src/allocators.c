#include <allocators.h>

int absoluteUsedMemory = 0;
int usedMemory = 0;

void * __freertos_allocate(size_t size, void * state){
  (void) state;
  // printf("-- Alloc %d (prev: %d B)\n",size, xPortGetFreeHeapSize());
  absoluteUsedMemory += size;
  usedMemory += size;
  return pvPortMalloc(size);
}

void __freertos_deallocate(void * pointer, void * state){
  (void) state;
 if( NULL != pointer){
    // printf("-- Free %d (prev: %d B)\n",getBlockSize(pointer), xPortGetFreeHeapSize());
    usedMemory -= getBlockSize(pointer);
    vPortFree(pointer);
 }
}

void * __freertos_reallocate(void * pointer, size_t size, void * state){
  (void) state;
  // printf("-- Realloc %d -> %d (prev: %d B)\n",getBlockSize(pointer),size, xPortGetFreeHeapSize());
  if (NULL != pointer){
    absoluteUsedMemory += size;
    usedMemory += size;
    usedMemory -= getBlockSize(pointer);
    return pvPortRealloc(pointer,size);
  }
  else {
      absoluteUsedMemory += size;
      usedMemory += size;
      return pvPortMalloc(size);
  }
}

void * __freertos_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state){
  (void) state;
  // printf("-- Calloc %d x %d = %d -> (prev: %d B)\n",number_of_elements,size_of_element, number_of_elements*size_of_element, xPortGetFreeHeapSize());
  absoluteUsedMemory += number_of_elements*size_of_element;
  usedMemory += number_of_elements*size_of_element;
  return pvPortCalloc(number_of_elements,size_of_element);
}
