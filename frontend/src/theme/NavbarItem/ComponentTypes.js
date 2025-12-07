import DefaultNavbarItem from "@theme/NavbarItem/DefaultNavbarItem";
import DropdownNavbarItem from "@theme/NavbarItem/DropdownNavbarItem";
import LocaleDropdownNavbarItem from "@theme/NavbarItem/LocaleDropdownNavbarItem";
import SearchNavbarItem from "@theme/NavbarItem/SearchNavbarItem";
import HtmlNavbarItem from "@theme/NavbarItem/HtmlNavbarItem";
import DocNavbarItem from "@theme/NavbarItem/DocNavbarItem";
import DocSidebarNavbarItem from "@theme/NavbarItem/DocSidebarNavbarItem";
import DocsVersionNavbarItem from "@theme/NavbarItem/DocsVersionNavbarItem";
import DocsVersionDropdownNavbarItem from "@theme/NavbarItem/DocsVersionDropdownNavbarItem";

// IMPORT YOUR BUTTON
import AuthButton from "@site/src/components/Auth/AuthButton"; // Adjust path if it's in components
import UrduButton from "@site/src/components/UrduButton"; // 🟢 Check this path!
import LevelButton from "@site/src/components/LevelButton";

const ComponentTypes = {
  default: DefaultNavbarItem,
  localeDropdown: LocaleDropdownNavbarItem,
  search: SearchNavbarItem,
  dropdown: DropdownNavbarItem,
  html: HtmlNavbarItem,
  doc: DocNavbarItem,
  docSidebar: DocSidebarNavbarItem,
  docsVersion: DocsVersionNavbarItem,
  docsVersionDropdown: DocsVersionDropdownNavbarItem,

  // REGISTER YOUR CUSTOM COMPONENT
  "custom-authButton": AuthButton,
  "custom-urduButton": UrduButton, // 🟢 This key matches docusaurus.config.ts
  "custom-levelButton": LevelButton,
};

export default ComponentTypes;
