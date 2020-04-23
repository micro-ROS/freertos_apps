#ifndef _MAIN_H_
#define _MAIN_H_

void appMain(void);
void *pvPortRealloc( void *pv, size_t xWantedSize );
void *pvPortCalloc( size_t num, size_t xWantedSize );
size_t getBlockSize( void *pv );

static int absoluteUsedMemory;
static int usedMemory;

inline static void * __crazyflie_allocate(size_t size, void * state)
{
  (void) state;
  void * ptr;
  absoluteUsedMemory += size;
  usedMemory += size;
  ptr = pvPortMalloc(size);
  // DEBUG_PRINT("-- Alloc %d (prev: %d B) at %X\n",size, xPortGetFreeHeapSize(),ptr);
  return ptr;
}

inline static void __crazyflie_deallocate(void * pointer, void * state)
{
  (void) state;
  // DEBUG_PRINT("-- Free %d (prev: %d B) at %X\n",getBlockSize(pointer), xPortGetFreeHeapSize(),pointer);
  usedMemory -= getBlockSize(pointer);
  vPortFree(pointer);
}

inline static void * __crazyflie_reallocate(void * pointer, size_t size, void * state)
{
  (void) state;
  void * ptr;
  absoluteUsedMemory += size;
  usedMemory += size;
  usedMemory -= getBlockSize(pointer);
  ptr = pvPortRealloc(pointer,size);
  // DEBUG_PRINT("-- Realloc %d -> %d (prev: %d B) from %X to %X\n",getBlockSize(pointer),size, xPortGetFreeHeapSize(),pointer, ptr);
  return ptr;
}

inline static void * __crazyflie_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
{
  (void) state;
  void * ptr;
  absoluteUsedMemory += number_of_elements*size_of_element;
  usedMemory += number_of_elements*size_of_element;
  ptr = pvPortCalloc(number_of_elements,size_of_element);
  // DEBUG_PRINT("-- Calloc %d x %d = %d -> (prev: %d B) at %X\n",number_of_elements,size_of_element, number_of_elements*size_of_element, xPortGetFreeHeapSize(), ptr);
  return ptr;
}

#endif // _MAIN_H_
