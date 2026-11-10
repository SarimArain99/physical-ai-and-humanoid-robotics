import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "A free 13-week textbook on robots that see, think, and act.",
  favicon: "img/favicon.ico",

  url: "https://physical-ai-and-humanoid-robotics-sarim.vercel.app",
  baseUrl: "/",

  organizationName: "SarimArain99",
  projectName: "physical-ai-and-humanoid-robotics",

  // Broken links must fail the build — silent 404s are how 12 dead chapter
  // cards shipped in the first place.
  onBrokenLinks: "throw",
  onBrokenAnchors: "throw",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  // Local full-text search (no Algolia account needed).
  themes: [
    [
      "@easyops-cn/docusaurus-search-local",
      {
        hashed: true,
        indexDocs: true,
        indexBlog: false,
        language: ["en"],
      },
    ],
  ],

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          editUrl:
            "https://github.com/SarimArain99/physical-ai-and-humanoid-robotics/tree/main/frontend",
        },
        blog: false, // No blog — the footer must not link to one.
        theme: {
          customCss: [
            "./src/css/custom.css",
            "./src/css/fonts.css",
            "./src/css/theme.css",
            "./src/css/animations.css",
          ],
        },
      } satisfies Preset.Options,
    ],
  ],

  // Site-wide description so every page carries a meta description.
  headTags: [
    {
      tagName: "meta",
      attributes: {
        name: "description",
        content:
          "A free 13-week textbook on Physical AI and humanoid robotics: ROS 2, simulation, NVIDIA Isaac, and humanoid control, with real code and quizzes.",
      },
    },
  ],

  themeConfig: {
    image: "img/social-card.svg",
    colorMode: {
      // Dark first: the book reads as a dark-premium textbook. One toggle
      // remains (the custom ThemeToggle in the navbar); the built-in
      // Docusaurus switch is hidden by the null ColorModeToggle swizzle
      // instead of disableSwitch — Docusaurus deletes the stored theme on
      // every mount when disableSwitch is true, so a visitor's Light/Dark
      // choice would never survive a page load (found live 2026-11-10).
      defaultMode: "dark",
      respectPrefersColorScheme: false,
    },
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      logo: {
        alt: "Physical AI & Humanoid Robotics",
        src: "img/logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Start Course",
        },
        {
          type: "custom-levelButton",
          position: "right",
        },
        {
          type: "custom-urduButton",
          position: "right",
        },
        // Theme toggle
        {
          type: "custom-themeToggle",
          position: "right",
        },
        {
          type: "custom-authButton",
          position: "right",
        },
        {
          href: "https://github.com/SarimArain99/physical-ai-and-humanoid-robotics",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Docs",
          items: [
            {
              label: "Introduction",
              to: "/docs/intro",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "Github",
              href: "https://github.com/SarimArain99/",
            },
            {
              label: "Instagram",
              href: "https://www.instagram.com/dev_sarim_arain/",
            },
            {
              label: "LinkedIn",
              href: "https://www.linkedin.com/in/muhammad-sarim-arain-b3795a2ba/",
            },
          ],
        },
        {
          title: "More",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/SarimArain99/physical-ai-and-humanoid-robotics",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook, by Muhammad Sarim.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
