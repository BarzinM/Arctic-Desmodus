#include <curses.h>

int main()
  {
  char users_name[ 100 ];

  initscr();
  (void)echo();

  addstr( "What is your name> " );
  refresh();
  getnstr( users_name, sizeof( users_name ) - 1 );

  /* Here is where we clear the screen.                  */
  /* (Remember, when using Curses, no change will appear */
  /* on the screen until <b>refresh</b>() is called.     */
  clear();

  printw( "Greetings and salutations %s!\n", users_name );
  refresh();

  printw( "\n\n\nPress ENTER to quit." );
  refresh();
  getnstr( users_name, sizeof( users_name ) - 1 );

  endwin();
  return 0;
  }