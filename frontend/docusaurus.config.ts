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

  themeConfig: {
    image: "img/docusaurus-social-card.jpg",
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      logo: {
        alt: "Physical AI Logo",
        src: "img/logo.png",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Start Course",
        },
        {
          type: "custom-levelButton", // 🟢 Add to Navbar
          position: "right",
        },
        // 🟢 Adds the Urdu Toggle
        {
          type: "custom-urduButton",
          position: "right",
        },
        // Theme toggle
        {
          type: "custom-themeToggle",
          position: "right",
        },
        // ✅ Your Login Link is Correct here
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
              label: "Tutorial",
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
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook, by Muhammad Sarim. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
