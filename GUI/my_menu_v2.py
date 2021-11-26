import pygame
from snake2 import game_loop

pygame.init()
Width = 480
Height = 480
pygame.display.set_caption("Snake")
screen = pygame.display.set_mode((Width, Height))

# colors
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
gray = (100, 100, 100)
yellow = (255, 255, 0)

# fonts
font1 = pygame.font.SysFont("monospace", 64, bold=True)
font2 = pygame.font.SysFont("ginger", 50, bold=True)
font3 = pygame.font.SysFont("Comic Sans MS", 45, bold=False)
font4 = pygame.font.SysFont("Arial", 32, bold=False)


def draw_text(text, font, color, offset1, offset2):
    title = font.render(text, True, color)
    title_rect = title.get_rect()
    title_rect.center = (Width // 2 - offset2, Height // 2 + offset1)
    screen.blit(title, title_rect)
    return title_rect


class button():
    def __init__(self, color, x, y, width, height, text=''):
        self.color = color
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.text = text

    def draw(self, screen, font, offset1, offset2):
        global text_rect
        but = pygame.Rect(self.x, self.y, self.width, self.height)
        pygame.draw.rect(screen, self.color, but, 2, 10)
        if self.text != '':
            text = font.render(self.text, 1, self.color)
            text_rect = text.get_rect()
            text_rect.center = (Width // 2 - offset2, Height // 2 + offset1)
            screen.blit(text, text_rect)

        return text_rect

    def isOver(self, pos):
        if self.x < pos[0] < self.x + self.width:
            if self.y < pos[1] < self.y + self.height:
                return True

        return False


def play():
    running = True
    while running:
        screen.fill(black)
        back_rect = draw_text("Back", font2, blue, 50, 0)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if back_rect.collidepoint(event.pos):
                    running = False
                    main_menu()
        pygame.display.update()


def credits():
    running = True
    while running:
        screen.fill(black)
        draw_text("Created by", font4, gray, 10, 75)
        draw_text("BlueRay", font4, white, 10, -80)

        # handling events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    main_menu()

        pygame.display.update()


def main_menu():
    FPS = 60
    clock = pygame.time.Clock()
    button1 = button(yellow, 130, 245, 220, 50, "Play")
    button2 = button(yellow, 130, 315, 220, 50, "Credits")

    running = True
    while running:
        c, p = 0, 0
        screen.fill((7, 7, 7))
        draw_text("Domino Game", font1, white, -150, 0)

        play_rect = button1.draw(screen, font2, 30, 0)
        cred_rect = button2.draw(screen, font2, 100, 0)

        # handling events
        for event in pygame.event.get():
            pos = pygame.mouse.get_pos()

            if event.type == pygame.QUIT:
                running = False
                pygame.exit()

            if event.type == pygame.MOUSEMOTION:
                if button1.isOver(pos):
                    button1.color = gray
                elif not button1.isOver(pos):
                    button1.color = yellow
                if button2.isOver(pos):
                    button2.color = gray
                elif not button2.isOver(pos):
                    button2.color = yellow

            if event.type == pygame.MOUSEBUTTONDOWN:
                if play_rect.collidepoint(event.pos):
                    play()
                elif cred_rect.collidepoint(event.pos):
                    credits()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_DOWN:
                    button1.color = yellow
                    button2.color = gray
                    clock.tick(FPS)

                elif event.key == pygame.K_UP:
                    button1.color = gray
                    button2.color = yellow
                    clock.tick(FPS)

        pygame.display.update()


main_menu()
