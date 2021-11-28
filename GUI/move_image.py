"""
https://www.pygame.org/docs/tut/MoveIt.html
"""
import pygame

screen = pygame.display.set_mode((640, 480))

# the load function takes a filename and returns a new Surface with the loaded image
background = pygame.image.load("table.jpg").convert()    # converts to the same pixel format as our display
player = pygame.image.load("cards/[6,6].png").convert()

# scale image (https://stackoverflow.com/questions/43046376/how-to-change-an-image-size-in-pygame)
player = pygame.transform.scale(player, (50, 100))

run = True
while run:
    # blit basically copies a pixel config. from one SOURCE to another
    # in other words it prints/draws image or text on screen
    screen.blit(background, (0, 0))
    position = player.get_rect()                        # return (x,y) position of image/text
    screen.blit(player, position)

    for x in range(60):                                 # animate 60 frames
        screen.blit(background, position, position)     # erase
        position = position.move(2, 2)                  # move player
        screen.blit(player, position)                   # draw new player
        pygame.display.update()                         # and show it all
        pygame.time.delay(100)

    for event in pygame.event.get():
        if event.type in (pygame.QUIT, pygame.KEYDOWN):
            run = False
    pygame.display.update()
