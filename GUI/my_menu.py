import pygame

pygame.init()
Width = 1000
Height = 1000
pygame.display.set_caption("Domino")
screen = pygame.display.set_mode((Width, Height))

# colors
black = (0, 0, 0)
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
gray = (100, 100, 100)

# fonts
font1 = pygame.font.SysFont("Comic sans ms", 96, bold=True)
font2 = pygame.font.Font(None, 64)


def draw_text(text, font, color, offset):
    title = font.render(text, True, color)
    title_rect = title.get_rect()
    title_rect.center = (Width // 2, Height // 2 + offset)
    screen.blit(title, title_rect)
    return title_rect


def play():
    run = True
    while run:
        screen.fill(black)
        draw_text("Welcome to Domino!", font2, green, -300)
        back_rect = draw_text("Back", font2, blue, 50)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    run = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if back_rect.collidepoint(event.pos):
                    run = False
                    main_menu()
        pygame.display.update()


def settings():
    run = True
    while run:
        screen.fill(black)
        draw_text("Change properties!", font2, green, -300)
        back_rect = draw_text("Back", font2, blue, 50)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    run = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if back_rect.collidepoint(event.pos):
                    run = False
                    main_menu()
        pygame.display.update()


def main_menu():
    run = True
    # menu_bg = pygame.image.load("bg_menu.jpg")
    while run:
        # screen.blit(menu_bg, (0,0))
        screen.fill(black)
        button_0 = pygame.Rect(Width//4, 200, Width//2, 50)
        pygame.draw.rect(screen, gray, button_0)
        draw_text("DOMINO", font1, white, -300)

        play_rect = draw_text(" Play", font2, white, 100)
        button_1 = pygame.Rect(380, 570, 260, 60)
        pygame.draw.rect(screen, gray, button_1, 2, 10)

        sett_rect = draw_text("Settings", font2, white, 250)
        button_2 = pygame.Rect(380, 718, 260, 60)
        pygame.draw.rect(screen, gray, button_2, 1, 10)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if play_rect.collidepoint(event.pos):
                    play()
                if sett_rect.collidepoint(event.pos):
                    settings()

        pygame.display.update()


main_menu()