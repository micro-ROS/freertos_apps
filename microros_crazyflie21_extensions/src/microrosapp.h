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
  DEBUG_PRINT("-- Alloc %d (prev: %d B)\n",size, xPortGetFreeHeapSize());
  absoluteUsedMemory += size;
  usedMemory += size;
  return pvPortMalloc(size);
}

inline static void __crazyflie_deallocate(void * pointer, void * state)
{
  (void) state;
  DEBUG_PRINT("-- Free %d (prev: %d B)\n",getBlockSize(pointer), xPortGetFreeHeapSize());
  usedMemory -= getBlockSize(pointer);
  vPortFree(pointer);
}

inline static void * __crazyflie_reallocate(void * pointer, size_t size, void * state)
{
  (void) state;
  DEBUG_PRINT("-- Realloc %d -> %d (prev: %d B)\n",getBlockSize(pointer),size, xPortGetFreeHeapSize());
  absoluteUsedMemory += size;
  usedMemory += size;
  usedMemory -= getBlockSize(pointer);
  return pvPortRealloc(pointer,size);
}

inline static void * __crazyflie_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
{
  (void) state;
  DEBUG_PRINT("-- Calloc %d x %d = %d -> (prev: %d B)\n",number_of_elements,size_of_element, number_of_elements*size_of_element, xPortGetFreeHeapSize());
  absoluteUsedMemory += number_of_elements*size_of_element;
  usedMemory += number_of_elements*size_of_element;
  return pvPortCalloc(number_of_elements,size_of_element);
}

#endif // _MAIN_H_
