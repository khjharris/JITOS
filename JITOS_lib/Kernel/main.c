

int main( void ) 
{
    char *string = "Hello World\n";
    while( *string )
    {
        //Writing hello world to usart2 memory 
        *( volatile char *)0x40004400 = *string; 
        string++;
    }

    while(1);
    return 0;
}
