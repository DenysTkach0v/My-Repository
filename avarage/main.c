#include <stdio.h>

int main(void) {
    char input[6];  // Массив для хранения 6 символов
    int count = 0;  // Счетчик для отслеживания количества считанных символов

    printf("Enter 6 characters: ");
    fflush(stdout);

    // Чтение символов по одному и игнорирование пробелов
    while (count < 6) {
        char ch;
        scanf("%c", &ch);  // Чтение одного символа, игнорируя пробелы перед ним

        if (ch != ' ') {    // Если символ не пробел, сохраняем его
            input[count] = ch;
            count++;
        }
    }

    printf("ASCII codes:\n");
    for (int i = 0; i < 6; i++) {
        printf("'%c' : %d\n", input[i], (int)input[i]);
    }

    return 0;
}
